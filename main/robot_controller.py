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
        """Initialize the robot and vision systems."""
        try:
            # Initialize robot
            self.status_queue.put("Initializing robot...")
            try:
                self.bot = Wafflebot(automatic_mode=True, detect_collisions=True)
                self.status_queue.put("Robot hardware initialized successfully")
            except Exception as robot_error:
                self.status_queue.put(f"ERROR initializing robot hardware: {str(robot_error)}")
                raise robot_error

            # Initialize vision and state
            self.status_queue.put("Initializing vision system...")
            try:
                self.vision = Vision()
                self.status_queue.put("Vision system initialized successfully")
            except Exception as vision_error:
                self.status_queue.put(f"ERROR initializing vision system: {str(vision_error)}")
                raise vision_error

            self.state = CurrentState(State.SLEEP)

            # Send status updates
            self.status_queue.put("Robot initialized in SLEEP state - Waiting for user input")
            self.status_queue.put("Robot and vision systems initialized - Ready for operation")

            return True
        except Exception as e:
            self.status_queue.put(f"Error initializing robot: {str(e)}")
            handle_error(e)
            return False

    def process_command(self, command: str, data: Optional[str] = None):
        """Process a command from the GUI.

        Args:
            command: The command to process
            data: Optional data associated with the command
        """
        try:
            if command == "CHANGE_STATE":
                self._change_state(data)
            elif command == "EMERGENCY_STOP":
                self._emergency_stop()
            elif command == "STOP_ROBOT":
                self._stop_robot()
            elif command == "MAKE_WAFFLE":
                self._make_waffle()
            elif command == "EXIT":
                self._exit()
        except Exception as e:
            self.status_queue.put(f"Error processing command {command}: {str(e)}")

    def _change_state(self, state_name: str):
        """Change the robot's state.

        Args:
            state_name: The name of the state to change to
        """
        try:
            # Check if robot and vision are initialized
            if self.bot is None or self.vision is None:
                self.status_queue.put("ERROR: Robot or vision system not initialized. Reinitializing...")
                if not self.initialize():
                    self.status_queue.put("ERROR: Failed to initialize robot. Cannot change state.")
                    return
                self.status_queue.put("Reinitialization successful. Continuing with state change.")

            # Get the new state from the enum
            try:
                new_state = State[state_name]
            except (KeyError, ValueError):
                self.status_queue.put(f"ERROR: Invalid state: {state_name}")
                return

            # Set the new state
            self.state.set(new_state)
            self.status_queue.put(f"State changed to {new_state.name}")

            # Enable state processing
            self.process_states = True

            # Immediately execute the state action
            self.status_queue.put(f"Executing state action for {new_state.name}...")
            try:
                # Set the processing flag
                Robot._processing_state_action = True
                self._execute_state_action(new_state)
                self.status_queue.put(f"State action for {new_state.name} executed successfully")
                Robot._processing_state_action = False
            except Exception as e:
                error_message = f"Error executing state action for {new_state.name}: {str(e)}"
                self.status_queue.put(f"ERROR: {error_message}")
                print(error_message)
                import traceback
                print(traceback.format_exc())
                self.state.set(State.ERROR)
                Robot._processing_state_action = False

        except Exception as e:
            error_message = f"Error changing state to {state_name}: {str(e)}"
            self.status_queue.put(f"ERROR: {error_message}")
            print(error_message)
            import traceback
            print(traceback.format_exc())

    def _emergency_stop(self):
        """Handle emergency stop command."""
        self.status_queue.put("EMERGENCY STOP ACTIVATED")
        self.bot.safe_stop(slow=True)
        # Stop processing states after emergency stop
        self.process_states = False
        # Reset to SLEEP state
        self.state.set(State.SLEEP)

    def _stop_robot(self):
        """Handle stop robot command."""
        self.status_queue.put("Robot stopped")
        self.bot.safe_stop()
        # Stop processing states after robot stop
        self.process_states = False
        # Reset to SLEEP state
        self.state.set(State.SLEEP)

    def _make_waffle(self):
        """Handle make waffle command."""
        self.status_queue.put("Starting waffle making sequence")

        # Check if robot is properly initialized
        if self.bot is None or self.vision is None:
            self.status_queue.put("ERROR: Robot or vision system not initialized. Reinitializing...")
            if not self.initialize():
                self.status_queue.put("ERROR: Failed to initialize robot. Cannot make waffle.")
                return
            self.status_queue.put("Reinitialization successful. Continuing with waffle making.")

        # Set the state to start the waffle making sequence
        self.state.set(State.SLEEP)
        # Enable state processing to start the sequence
        self.process_states = True
        self.status_queue.put("State set to SLEEP, beginning waffle sequence...")

        # Immediately execute the state action
        try:
            self.status_queue.put("Executing SLEEP state action...")
            # Set the processing flag
            Robot._processing_state_action = True
            self._execute_state_action(State.SLEEP)
            self.status_queue.put("SLEEP state action executed successfully")
            Robot._processing_state_action = False
        except Exception as e:
            error_message = f"Error executing SLEEP state action: {str(e)}"
            self.status_queue.put(f"ERROR: {error_message}")
            print(error_message)
            import traceback
            print(traceback.format_exc())
            self.state.set(State.ERROR)
            Robot._processing_state_action = False

    def _exit(self):
        """Handle exit command."""
        self.status_queue.put("Shutting down...")
        self.bot.safe_stop()
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
        match current_state:
            case State.REST:
                self.status_queue.put("Robot is resting")
                rest(self.state, self.bot)

            case State.HOME:
                self.status_queue.put("Moving to home position")
                home(self.state, self.bot, self.vision)

            case State.SLEEP:
                self.status_queue.put("Robot is in sleep state")
                start(self.state, self.bot, self.vision)

            case State.OPEN_IRON:
                self.status_queue.put("Opening waffle iron")
                open_iron(self.state, self.bot, self.vision)

            case State.PICK_UP_SPRAY:
                self.status_queue.put("Picking up spray")
                pick_up_spray(self.state, self.bot, self.vision)

            case State.SPRAY:
                self.status_queue.put("Spraying waffle iron")
                spray(self.state, self.bot)

            case State.PUT_DOWN_SPRAY:
                self.status_queue.put("Putting down spray")
                put_down_spray(self.state, self.bot, self.vision)

            case State.PICK_UP_LADLE:
                self.status_queue.put("Picking up ladle")
                pick_up_ladle(self.state, self.bot)

            case State.CLOSE_IRON:
                self.status_queue.put("Closing waffle iron")
                close_iron(self.state, self.bot)

            case State.FUN_TIME:
                self.status_queue.put("Cooking waffle")
                fun_time(self.state, self.bot)

            case State.POUR_BATTER:
                self.status_queue.put("Pouring batter")
                pour_batter(self.state, self.bot)

            case State.RETURN_LADLE:
                self.status_queue.put("Returning ladle")
                return_ladle(self.state, self.bot, self.vision)

            case State.OPEN_IRON2:
                self.status_queue.put("Opening iron to check waffle")
                open_iron2(self.state, self.bot)

            case State.RETURN_STICK:
                self.status_queue.put("Returning stick")
                return_stick(self.state, self.bot, self.vision)

            case State.PICK_UP_WAFFLE:
                self.status_queue.put("Picking up waffle")
                pick_up_waffle(self.state, self.bot, self.vision)
                # Increment waffle counter
                self.waffle_counter += 1

            case State.ERROR:
                self.status_queue.put("ERROR: Robot needs attention!")
                error(self.state, self.bot)

            case _:
                self.status_queue.put("Unknown state encountered")
                self.bot.safe_stop(slow=True)

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
                    command, data = self.command_queue.get(block=False)
                    self.status_queue.put(f"Received command: {command}")
                    self.process_command(command, data)

                    # Check for exit command
                    if command == "EXIT":
                        break
                except queue.Empty:
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
