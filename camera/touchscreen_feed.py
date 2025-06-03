"""
Touchscreen-friendly camera feed interface using CustomTkinter.
"""

import cv2
import numpy as np
import customtkinter as ctk
from PIL import Image
import threading
import time
from typing import Optional, Callable


class TouchscreenCameraFeed:
    """Touchscreen-friendly camera feed window with large controls."""
    
    def __init__(self, vision_system, status_callback: Optional[Callable] = None):
        """Initialize the touchscreen camera feed.
        
        Args:
            vision_system: The vision system instance
            status_callback: Optional callback for status updates
        """
        self.vision = vision_system
        self.status_callback = status_callback
        self.running = False
        self.window = None
        self.video_label = None
        self.feed_thread = None
        
        # Camera feed settings
        self.detect_hands = False
        self.detect_gestures = False
        self.draw_cubes = True
        
        # Window settings
        self.window_width = 1200
        self.window_height = 900
        
    def start_feed(self):
        """Start the camera feed window."""
        if self.running:
            return
            
        self.running = True
        self._create_window()
        self._start_video_thread()
        
    def stop_feed(self):
        """Stop the camera feed."""
        self.running = False
        if self.window:
            self.window.destroy()
            self.window = None
            
    def _create_window(self):
        """Create the touchscreen-friendly window."""
        # Set appearance
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        
        # Create main window
        self.window = ctk.CTkToplevel()
        self.window.title("Camera Feed - Touch Controls")
        self.window.geometry(f"{self.window_width}x{self.window_height}")
        self.window.attributes("-topmost", True)  # Keep on top
        
        # Configure grid
        self.window.grid_columnconfigure(0, weight=1)
        self.window.grid_rowconfigure(0, weight=1)
        
        # Main container
        main_frame = ctk.CTkFrame(self.window)
        main_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_rowconfigure(1, weight=1)
        
        # Title and controls frame
        controls_frame = ctk.CTkFrame(main_frame)
        controls_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=10)
        controls_frame.grid_columnconfigure(1, weight=1)
        
        # Title
        title_label = ctk.CTkLabel(
            controls_frame,
            text="Camera Feed",
            font=ctk.CTkFont(size=24, weight="bold")
        )
        title_label.grid(row=0, column=0, columnspan=4, pady=10)
        
        # Control buttons (large for touch)
        button_config = {
            "height": 60,
            "width": 150,
            "font": ctk.CTkFont(size=16, weight="bold"),
            "corner_radius": 10
        }
        
        # Close button (red)
        close_btn = ctk.CTkButton(
            controls_frame,
            text="✕ Close",
            fg_color="#DC2626",
            hover_color="#B91C1C",
            command=self._close_window,
            **button_config
        )
        close_btn.grid(row=1, column=0, padx=10, pady=5)
        
        # Hand detection toggle
        self.hand_btn = ctk.CTkButton(
            controls_frame,
            text="👋 Hands: OFF",
            fg_color="#059669",
            hover_color="#047857",
            command=self._toggle_hands,
            **button_config
        )
        self.hand_btn.grid(row=1, column=1, padx=10, pady=5)
        
        # Gesture detection toggle
        self.gesture_btn = ctk.CTkButton(
            controls_frame,
            text="✋ Gestures: OFF",
            fg_color="#7C3AED",
            hover_color="#6D28D9",
            command=self._toggle_gestures,
            **button_config
        )
        self.gesture_btn.grid(row=1, column=2, padx=10, pady=5)
        
        # Marker cubes toggle
        self.cubes_btn = ctk.CTkButton(
            controls_frame,
            text="📦 Cubes: ON",
            fg_color="#EA580C",
            hover_color="#DC2626",
            command=self._toggle_cubes,
            **button_config
        )
        self.cubes_btn.grid(row=1, column=3, padx=10, pady=5)
        
        # Video display frame
        video_frame = ctk.CTkFrame(main_frame)
        video_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
        video_frame.grid_columnconfigure(0, weight=1)
        video_frame.grid_rowconfigure(0, weight=1)
        
        # Video label
        self.video_label = ctk.CTkLabel(
            video_frame,
            text="Starting camera feed...",
            font=ctk.CTkFont(size=18)
        )
        self.video_label.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)
        
        # Status label
        self.status_label = ctk.CTkLabel(
            main_frame,
            text="Camera feed starting...",
            font=ctk.CTkFont(size=14)
        )
        self.status_label.grid(row=2, column=0, pady=5)
        
        # Handle window close
        self.window.protocol("WM_DELETE_WINDOW", self._close_window)
        
    def _start_video_thread(self):
        """Start the video feed thread."""
        self.feed_thread = threading.Thread(target=self._video_loop, daemon=True)
        self.feed_thread.start()
        
    def _video_loop(self):
        """Main video processing loop."""
        try:
            while self.running and self.window:
                try:
                    # Get frame from vision system
                    imglist = self.vision.run_once(
                        return_image=True,
                        draw_cubes=self.draw_cubes,
                        detect_hands=self.detect_hands,
                        detect_gestures=self.detect_gestures
                    )

                    if imglist and len(imglist) > 0:
                        # Use the first camera image
                        img = list(imglist)[0]
                        if img is not None:
                            self._update_video_display(img)

                except RuntimeError as e:
                    # Handle camera connection issues gracefully
                    if "camera not connected" in str(e).lower():
                        self._update_status("Camera connection issue - retrying...")
                        time.sleep(1)  # Wait before retrying
                        continue
                    else:
                        raise e

                time.sleep(0.033)  # ~30 FPS

        except Exception as e:
            self._update_status(f"Error in video feed: {str(e)}")
            print(f"Video loop error: {e}")
            import traceback
            print(traceback.format_exc())
            
    def _update_video_display(self, cv_image):
        """Update the video display with new frame."""
        if not self.window or not self.video_label:
            return
            
        try:
            # Resize image to fit display while maintaining aspect ratio
            display_width = 800
            display_height = 600
            
            h, w = cv_image.shape[:2]
            aspect_ratio = w / h
            
            if aspect_ratio > display_width / display_height:
                new_width = display_width
                new_height = int(display_width / aspect_ratio)
            else:
                new_height = display_height
                new_width = int(display_height * aspect_ratio)
                
            resized_image = cv2.resize(cv_image, (new_width, new_height))
            
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
            
            # Convert to PIL Image
            pil_image = Image.fromarray(rgb_image)

            # Convert to CTkImage for better scaling
            ctk_image = ctk.CTkImage(light_image=pil_image, dark_image=pil_image, size=(new_width, new_height))

            # Update label
            self.video_label.configure(image=ctk_image, text="")
            self.video_label.image = ctk_image  # Keep a reference
            
        except Exception as e:
            print(f"Error updating video display: {e}")
            
    def _update_status(self, message: str):
        """Update the status message."""
        if self.status_label:
            self.status_label.configure(text=message)
        if self.status_callback:
            self.status_callback(message)
            
    def _toggle_hands(self):
        """Toggle hand detection."""
        self.detect_hands = not self.detect_hands
        text = "👋 Hands: ON" if self.detect_hands else "👋 Hands: OFF"
        color = "#DC2626" if self.detect_hands else "#059669"
        self.hand_btn.configure(text=text, fg_color=color)
        self._update_status(f"Hand detection: {'ON' if self.detect_hands else 'OFF'}")
        
    def _toggle_gestures(self):
        """Toggle gesture detection."""
        self.detect_gestures = not self.detect_gestures
        text = "✋ Gestures: ON" if self.detect_gestures else "✋ Gestures: OFF"
        color = "#DC2626" if self.detect_gestures else "#7C3AED"
        self.gesture_btn.configure(text=text, fg_color=color)
        self._update_status(f"Gesture detection: {'ON' if self.detect_gestures else 'OFF'}")
        
    def _toggle_cubes(self):
        """Toggle marker cubes."""
        self.draw_cubes = not self.draw_cubes
        text = "📦 Cubes: ON" if self.draw_cubes else "📦 Cubes: OFF"
        color = "#DC2626" if self.draw_cubes else "#EA580C"
        self.cubes_btn.configure(text=text, fg_color=color)
        self._update_status(f"Marker cubes: {'ON' if self.draw_cubes else 'OFF'}")
        
    def _close_window(self):
        """Close the camera feed window."""
        self._update_status("Closing camera feed...")
        self.stop_feed()
