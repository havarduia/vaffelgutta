"""
Page Connector Module

This module contains the PageConnector class that connects the GUI pages to the robot controller.
"""

import queue
import threading
from typing import Callable

import customtkinter as ctk
from main.waffle_states.waffle_states import State


class Screen:
    """Connects the GUI pages to the robot controller."""
    
    def __init__(self, app, page_controller, command_sender: Callable, state_queue: queue.Queue, status_queue: queue.Queue):
        """Initialize the page connector.
        
        Args:
            app: The main application instance
            page_controller: The page controller instance
            command_sender: Function for sending commands to the robot controller
            state_queue: Queue for receiving state updates from the robot controller
            status_queue: Queue for receiving status messages from the robot controller
        """
        self.app = app
        self.page_controller = page_controller
        self.command_sender = command_sender
        self.state_queue = state_queue
        self.status_queue = status_queue
        
        # Initialize waffle counter
        self.waffle_counter = 0
        
        # Connect all pages
        self._connect_home_page()
        self._connect_dev_page()
        self._connect_emergency_page()
        self._connect_stats_page()
    
    def _connect_home_page(self):
        """Connect the home page to the robot controller."""
        home_page = self.page_controller.pages["Home"]
        
        # Override button commands to interact with the robot
        home_page.on_make_waffle = lambda: self.command_sender("MAKE_WAFFLE")
        home_page.change_state = lambda: self.page_controller.show_page("Dev Mode")  # Use Dev Mode for state changes
        home_page.stop_robot = lambda: self.command_sender("STOP_ROBOT")
        home_page.exit = lambda: (self.command_sender("EXIT"), self.app.after(1000, self.app.exit_application))
    
    def _connect_dev_page(self):
        """Connect the dev page to the robot controller."""
        dev_page = self.page_controller.pages["Dev Mode"]
        
        # Override the apply_state_change method to send commands to the robot
        dev_page.apply_state_change = lambda: self.command_sender("CHANGE_STATE", dev_page.state_var.get())
        
        # Override the reboot_system method
        dev_page.reboot_system = lambda: (
            self.command_sender("STOP_ROBOT"),
            self.status_queue.put("Rebooting system..."),
            self.app.after(2000, lambda: self.command_sender("MAKE_WAFFLE"))
        )
    
    def _connect_emergency_page(self):
        """Connect the emergency page to the robot controller."""
        emergency_page = self.page_controller.pages["Emergency"]
        
        # Override the reset_robot method
        emergency_page.reset_robot = lambda: (
            self.command_sender("CHANGE_STATE", "SLEEP"),
            self.status_queue.put("Resetting robot to SLEEP state..."),
            self.page_controller.show_page("Home")
        )
        
        # Override the exit_program method
        emergency_page.exit_program = lambda: (
            self.command_sender("EXIT"),
            self.app.after(1000, self.app.exit_application)
        )
    
    def _connect_stats_page(self):
        """Connect the stats page to the robot controller."""
        stats_page = self.page_controller.pages["Stats"]
        
        # Override the refresh_status method
        stats_page.refresh_status = lambda: (
            stats_page.update_state(self.state_queue.get().name if not self.state_queue.empty() else "UNKNOWN"),
            stats_page.update_status(self.app.status_var.get()),
            stats_page.update_counter(self.waffle_counter)
        )
    
    def start_stats_updates(self, robot_controller):
        """Start periodic updates for the stats page.
        
        Args:
            robot_controller: The robot controller instance for getting the waffle counter
        """
        def update_stats():
            stats_page = self.page_controller.pages["Stats"]
            if self.page_controller.current_page == stats_page:
                try:
                    current_state = self.state_queue.get(block=False)
                    stats_page.update_state(current_state.name)
                    
                    # Update waffle counter
                    self.waffle_counter = robot_controller.get_waffle_counter()
                    stats_page.update_counter(self.waffle_counter)
                    
                    stats_page.update_status(self.app.status_var.get())
                except queue.Empty:
                    pass
            self.app.after(2000, update_stats)
        
        # Start the periodic update
        self.app.after(2000, update_stats)
