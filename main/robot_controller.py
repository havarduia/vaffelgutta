"""
Robot Controller Module

This module contains the RobotController class that manages the robot's state and actions.
"""

import queue
import threading
import time
from typing import Optional, Tuple

from camera.vision import Vision
from robot.tools.errorhandling import handle_error
from robot.robot_controllers.Wafflebot.Wafflebot import Wafflebot
from main.waffle_states.waffle_states import State, CurrentState
from main.state_transitions import *
from sys import exit as sysexit


class Robot:
    """Controls the robot's actions and state transitions."""

    def __init__(self, command_queue: queue.Queue, state_queue: queue.Queue, status_queue: queue.Queue, stop_event: threading.Event):
        """Initialize the robot controller.

        Args:
            command_queue: Queue for receiving commands from the GUI
            state_queue: Queue for sending state updates to the GUI
            status_queue: Queue for sending status messages to the GUI
            stop_event: Event for signaling the controller to stop
        """
        self.command_queue = command_queue
        self.state_queue = state_queue
        self.status_queue = status_queue
        self.stop_event = stop_event

        # Initialize robot components
        self.bot = None
        self.vision = None
        self.state = None

        # Control flags
        self.process_states = False
        self.waffle_counter = 0

    def initialize(self):
        """Initialize the vision system independently from robot hardware."""
        # Skip robot hardware initialization - only initialize vision system
        self.status_queue.put("Initializing vision system (robot hardware will be initialized only when needed)...")

        try:
            self.vision = Vision()
            self.status_queue.put("Vision system initialized successfully")
            self.state = CurrentState(State.SLEEP)
            self.status_queue.put("System ready - Camera feed available")
            return True
        except Exception as vision_error:
            self.status_queue.put(f"ERROR initializing vision system: {str(vision_error)}")
            self.vision = None
            return False

    def initialize_robot_hardware(self):
        """Initialize robot hardware separately (only when needed for robot operations)."""
        if self.bot is not None:
            return True  # Already initialized

        self.status_queue.put("Initializing robot hardware...")
        try:
            self.bot = Wafflebot(automatic_mode=False, detect_collisions=False)
            self.status_queue.put("Robot hardware initialized successfully")
            return True
        except Exception as robot_error:
            self.status_queue.put(f"ERROR: Robot hardware initialization failed: {str(robot_error)}")
            self.bot = None
            return False

    def process_command(self, command: str, data: Optional[str] = None):
        """Process a command from the GUI.

        Args:
            command: The command to process
            data: Optional data associated with the command
        """
        print(f"DEBUG: Robot controller received command: {command}, data: {data}")
        try:
            if command == "CHANGE_STATE":
                print(f"DEBUG: Processing CHANGE_STATE command with data: {data}")
                self._change_state(data)
            elif command == "EMERGENCY_STOP":
                print("DEBUG: Processing EMERGENCY_STOP command")
                self._emergency_stop()
            elif command == "STOP_ROBOT":
                print("DEBUG: Processing STOP_ROBOT command")
                self._stop_robot()
            elif command == "MAKE_WAFFLE":
                print("DEBUG: Processing MAKE_WAFFLE command")
                self._make_waffle()
            elif command == "SHOW_CAMERA_FEED":
                print("DEBUG: Processing SHOW_CAMERA_FEED command")
                self._show_camera_feed()
            elif command == "EXIT":
                print("DEBUG: Processing EXIT command")
                self._exit()
            else:
                print(f"DEBUG: Unknown command received: {command}")
        except Exception as e:
            error_message = f"Error processing command {command}: {str(e)}"
            print(f"DEBUG: {error_message}")
            import traceback
            print(traceback.format_exc())
            self.status_queue.put(error_message)

    def _change_state(self, state_name: str):
        """Change the robot's state.

        Args:
            state_name: The name of the state to change to
        """
        try:
            # Check if robot hardware is initialized for state changes
            if self.bot is None:
                self.status_queue.put("Robot hardware not initialized. Initializing robot hardware...")
                if not self.initialize_robot_hardware():
                    self.status_queue.put("ERROR: Failed to initialize robot hardware. Cannot change state.")
                    return
                self.status_queue.put("Robot hardware initialization successful. Continuing with state change.")

            # Check if vision system is available
            if self.vision is None:
                self.status_queue.put("ERROR: Vision system not available. Cannot change state.")
                return

            # Get the new state from the enum
            try:
                new_state = State[state_name]
            except (KeyError, ValueError):
                self.status_queue.put(f"ERROR: Invalid state: {state_name}")
                return

            # Set the new state
            self.state.set(new_state)
            self.status_queue.put(f"State changed to {new_state.name}")

            # Enable state processing - let the main loop handle execution
            self.process_states = True
            self.status_queue.put(f"State processing enabled for {new_state.name} - will be executed by main loop")

        except Exception as e:
            error_message = f"Error changing state to {state_name}: {str(e)}"
            self.status_queue.put(f"ERROR: {error_message}")
            print(error_message)
            import traceback
            print(traceback.format_exc())

    def _emergency_stop(self):
        """Handle emergency stop command."""
        self.status_queue.put("EMERGENCY STOP ACTIVATED")
        if self.bot is not None:
            self.bot.safe_stop(slow=True)
        else:
            self.status_queue.put("Robot hardware not connected - Emergency stop signal sent")
        # Stop processing states after emergency stop
        self.process_states = False
        # Reset to SLEEP state
        if self.state is not None:
            self.state.set(State.SLEEP)

    def _stop_robot(self):
        """Handle stop robot command."""
        self.status_queue.put("Robot stopped")
        if self.bot is not None:
            self.bot.safe_stop()
        else:
            self.status_queue.put("Robot hardware not connected - Stop signal sent")
        # Stop processing states after robot stop
        self.process_states = False
        # Reset to SLEEP state
        if self.state is not None:
            self.state.set(State.SLEEP)

    def _make_waffle(self):
        """Handle make waffle command."""
        self.status_queue.put("Starting waffle making sequence")

        # Check if robot hardware is initialized (vision should already be initialized)
        if self.bot is None:
            self.status_queue.put("Robot hardware not initialized. Initializing robot hardware...")
            if not self.initialize_robot_hardware():
                self.status_queue.put("ERROR: Failed to initialize robot hardware. Cannot make waffle.")
                return
            self.status_queue.put("Robot hardware initialization successful. Continuing with waffle making.")

        # Check if vision system is available
        if self.vision is None:
            self.status_queue.put("ERROR: Vision system not available. Cannot make waffle.")
            return

        # Set the state to start the waffle making sequence
        self.state.set(State.SLEEP)
        # Enable state processing to start the sequence
        self.process_states = True
        self.status_queue.put("State set to SLEEP, beginning waffle sequence...")
        self.status_queue.put("Waffle making sequence will be handled by main state machine loop")

    def _show_camera_feed(self):
        """Handle show camera feed command."""
        self.status_queue.put("Starting touchscreen camera feed...")

        # Check if vision system is initialized
        if self.vision is None:
            self.status_queue.put("Vision system not initialized. Attempting to initialize vision only...")
            try:
                self.vision = Vision()
                self.status_queue.put("Vision system initialized successfully.")
            except Exception as vision_error:
                self.status_queue.put(f"ERROR: Failed to initialize vision system: {str(vision_error)}")
                return

        try:
            # Import the touchscreen camera feed
            from camera.touchscreen_feed import TouchscreenCameraFeed

            # Create status callback to update main GUI
            def status_callback(message):
                self.status_queue.put(f"Camera Feed: {message}")

            # Start touchscreen-friendly camera feed
            def camera_feed_thread():
                try:
                    self.status_queue.put("Touchscreen camera feed started. Use touch controls to interact.")

                    # Create and start the touchscreen camera feed
                    camera_feed = TouchscreenCameraFeed(self.vision, status_callback)
                    camera_feed.start_feed()

                    # Keep the thread alive while the window is open
                    while camera_feed.running and camera_feed.window:
                        time.sleep(0.1)

                    self.status_queue.put("Camera feed closed.")
                except Exception as e:
                    self.status_queue.put(f"ERROR in camera feed: {str(e)}")
                    print(f"Camera feed error: {e}")
                    import traceback
                    print(traceback.format_exc())

            # Start the camera feed thread as a daemon thread
            camera_thread = threading.Thread(target=camera_feed_thread, daemon=True)
            camera_thread.start()

        except Exception as e:
            error_message = f"Error starting camera feed: {str(e)}"
            self.status_queue.put(f"ERROR: {error_message}")
            print(error_message)
            import traceback
            print(traceback.format_exc())

    def _exit(self):
        """Handle exit command."""
        self.status_queue.put("Shutting down...")
        try:
            if self.bot is not None:
                self.bot.safe_stop()
        finally:
            sysexit()
        # The main loop will check the stop_event and exit

    # Flag to track if we're currently processing a state action
    _processing_state_action = False

    def process_state(self):
        """Process the current state and execute the corresponding action."""
        current_state = self.state.get()
        self.state_queue.put(current_state)  # Update GUI with current state

        # Only process state transitions if explicitly started
        if not self.process_states:
            # If not processing states, update status to show we're waiting for user input
            if current_state == State.SLEEP:
                self.status_queue.put("Robot idle - Press 'Make Waffle' to start")
            return False

        # Check if robot and vision are initialized
        if self.bot is None or self.vision is None:
            self.status_queue.put("ERROR: Cannot process state - Robot or vision not initialized")
            self.process_states = False
            return False

        # Prevent concurrent state processing
        if Robot._processing_state_action:
            return False

        # Execute state action
        try:
            Robot._processing_state_action = True
            self.status_queue.put(f"Executing state action for {current_state.name}...")
            self._execute_state_action(current_state)
            Robot._processing_state_action = False
            return True
        except Exception as e:
            error_message = f"Error in state {current_state.name}: {str(e)}"
            self.status_queue.put(f"ERROR: {error_message}")
            # Print to console for debugging
            print(error_message)
            import traceback
            print(traceback.format_exc())
            self.state.set(State.ERROR)
            Robot._processing_state_action = False
            return False

    def _execute_state_action(self, current_state: State):
        """Execute the action corresponding to the current state.

        Args:
            current_state: The current state of the robot
        """
        print(f"DEBUG: Executing state action for {current_state.name}")
        self.status_queue.put(f"DEBUG: Executing state action for {current_state.name}")

        try:
            match current_state:
                case State.REST:
                    self.status_queue.put("Robot is resting")
                    print("DEBUG: Calling rest() function")
                    rest(self.state, self.bot)
                    print("DEBUG: rest() function completed")

                case State.HOME:
                    self.status_queue.put("Moving to home position")
                    print("DEBUG: Calling home() function")
                    home(self.state, self.bot, self.vision)
                    print("DEBUG: home() function completed")

                case State.SLEEP:
                    self.status_queue.put("Robot is in sleep state")
                    print("DEBUG: Calling start() function")
                    start(self.state, self.bot, self.vision)
                    print("DEBUG: start() function completed")

                case State.OPEN_IRON:
                    self.status_queue.put("Opening waffle iron")
                    print("DEBUG: Calling open_iron() function")
                    open_iron(self.state, self.bot, self.vision)
                    print("DEBUG: open_iron() function completed")

                case State.PICK_UP_SPRAY:
                    self.status_queue.put("Picking up spray")
                    print("DEBUG: Calling pick_up_spray() function")
                    pick_up_spray(self.state, self.bot, self.vision)
                    print("DEBUG: pick_up_spray() function completed")

                case State.SPRAY:
                    self.status_queue.put("Spraying waffle iron")
                    print("DEBUG: Calling spray() function")
                    spray(self.state, self.bot)
                    print("DEBUG: spray() function completed")

                case State.PUT_DOWN_SPRAY:
                    self.status_queue.put("Putting down spray")
                    print("DEBUG: Calling put_down_spray() function")
                    put_down_spray(self.state, self.bot, self.vision)
                    print("DEBUG: put_down_spray() function completed")

                case State.PICK_UP_LADLE:
                    self.status_queue.put("Picking up ladle")
                    print("DEBUG: Calling pick_up_ladle() function")
                    pick_up_ladle(self.state, self.bot)
                    print("DEBUG: pick_up_ladle() function completed")

                case State.CLOSE_IRON:
                    self.status_queue.put("Closing waffle iron")
                    print("DEBUG: Calling close_iron() function")
                    close_iron(self.state, self.bot)
                    print("DEBUG: close_iron() function completed")

                case State.FUN_TIME:
                    self.status_queue.put("Cooking waffle")
                    print("DEBUG: Calling fun_time() function")
                    fun_time(self.state, self.bot)
                    print("DEBUG: fun_time() function completed")

                case State.POUR_BATTER:
                    self.status_queue.put("Pouring batter")
                    print("DEBUG: Calling pour_batter() function")
                    pour_batter(self.state, self.bot)
                    print("DEBUG: pour_batter() function completed")

                case State.RETURN_LADLE:
                    self.status_queue.put("Returning ladle")
                    print("DEBUG: Calling return_ladle() function")
                    return_ladle(self.state, self.bot, self.vision)
                    print("DEBUG: return_ladle() function completed")

                case State.OPEN_IRON2:
                    self.status_queue.put("Opening iron to check waffle")
                    print("DEBUG: Calling open_iron2() function")
                    open_iron2(self.state, self.bot)
                    print("DEBUG: open_iron2() function completed")

                case State.RETURN_STICK:
                    self.status_queue.put("Returning stick")
                    print("DEBUG: Calling return_stick() function")
                    return_stick(self.state, self.bot, self.vision)
                    print("DEBUG: return_stick() function completed")

                case State.PICK_UP_WAFFLE:
                    self.status_queue.put("Picking up waffle")
                    print("DEBUG: Calling pick_up_waffle() function")
                    pick_up_waffle(self.state, self.bot, self.vision)
                    print("DEBUG: pick_up_waffle() function completed")
                    # Increment waffle counter
                    self.waffle_counter += 1

                case State.ERROR:
                    self.status_queue.put("ERROR: Robot needs attention!")
                    print("DEBUG: Calling error() function")
                    error(self.state, self.bot)
                    print("DEBUG: error() function completed")

                case _:
                    self.status_queue.put("Unknown state encountered")
                    print("DEBUG: Unknown state, calling safe_stop()")
                    self.bot.safe_stop(slow=True)
                    print("DEBUG: safe_stop() completed")
        except Exception as e:
            error_message = f"Exception in _execute_state_action for {current_state.name}: {str(e)}"
            print(error_message)
            import traceback
            print(traceback.format_exc())
            self.status_queue.put(f"ERROR: {error_message}")
            raise  # Re-raise the exception to be caught by the caller

    def run(self):
        """Main control loop for the robot controller."""
        try:
            # Initialize robot and vision
            self.status_queue.put("Starting robot initialization...")
            if not self.initialize():
                self.status_queue.put("ERROR: Robot initialization failed. Please check the system and try again.")
                return

            self.status_queue.put("Robot initialization successful. Starting main control loop.")

            # Main control loop
            while not self.stop_event.is_set():
                # Check for commands
                try:
                    print("DEBUG: Checking command queue...")
                    command, data = self.command_queue.get(block=False)
                    print(f"DEBUG: Found command in queue: {command}, data: {data}")
                    self.status_queue.put(f"Received command: {command}")
                    self.process_command(command, data)

                    # Check for exit command
                    if command == "EXIT":
                        print("DEBUG: EXIT command received, breaking main loop")
                        break
                except queue.Empty:
                    # Print this message only occasionally to avoid flooding the console
                    if int(time.time()) % 5 == 0:  # Print every 5 seconds
                        print("DEBUG: No commands in queue")
                    pass

                # Process current state
                if not self.process_state():
                    # If not processing states, sleep a bit to reduce CPU usage
                    time.sleep(0.1)

            # Clean shutdown
            self.status_queue.put("Shutting down robot control thread...")
            if self.bot:
                try:
                    self.bot.safe_stop()
                    self.status_queue.put("Robot safely stopped")
                except Exception as stop_error:
                    self.status_queue.put(f"Error stopping robot: {str(stop_error)}")
            self.status_queue.put("Robot control thread terminated")

        except Exception as e:
            error_message = f"Critical error in robot control thread: {str(e)}"
            self.status_queue.put(f"ERROR: {error_message}")
            print(error_message)
            import traceback
            print(traceback.format_exc())
            handle_error(e)

    def get_waffle_counter(self) -> int:
        """Get the current waffle counter value.

        Returns:
            The number of waffles made
        """
        return self.waffle_counter
