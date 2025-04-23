"""
Threaded marker detection implementation.
"""

import time
import numpy as np
from threading import Thread, Lock


class MarkerDetectionThread(Thread):
    """Thread class for background marker detection"""
    
    def __init__(self, camera, aruco, coord_sys, jsonreader):
        Thread.__init__(self)
        self.daemon = True  # Thread will exit when main program exits
        self.camera = camera
        self.aruco = aruco
        self.coord_sys = coord_sys
        self.jsonreader = jsonreader
        
        # Shared data with locks
        self.lock = Lock()
        self.running = True
        self.draw_cubes = False
        self.allowed_tags = ()
        
        # Results
        self.latest_image = None
        self.latest_transformed = {}
        self.latest_raw = {}
        self.latest_timestamp = 0
        
        # Performance metrics
        self.fps = 0
        self.processing_time = 0
    
    def run(self):
        """Main thread loop"""
        frames_count = 0
        start_time = time.time()
        
        while self.running:
            try:
                # Get a frame
                color_frame = self.camera.get_color_frame()
                if color_frame is None:
                    time.sleep(0.01)  # Short sleep to prevent CPU hogging
                    continue
                
                # Process frame
                process_start = time.time()
                raw_poses, marker_img = self.aruco.estimate_poses(
                    image=color_frame, 
                    draw_cubes=self.draw_cubes
                )
                
                # Transform poses
                transformed = self.coord_sys.transform_poses(raw_poses)
                
                # Update shared data with lock
                with self.lock:
                    self.latest_image = marker_img if marker_img is not None else color_frame
                    self.latest_transformed = transformed
                    self.latest_raw = raw_poses
                    self.latest_timestamp = time.time()
                    self.processing_time = time.time() - process_start
                
                # Filter and write to file if needed
                allowed_tags = None
                with self.lock:
                    allowed_tags = self.allowed_tags
                
                if allowed_tags:
                    allowed_ints = set(int(t) for t in allowed_tags)
                    filtered = {tid: pose for tid, pose in transformed.items() if tid in allowed_ints}
                else:
                    filtered = transformed
                
                # Only write to file if we have data
                if filtered:
                    tags_data = {tid: mat for tid, mat in filtered.items()}
                    
                    # Add origin tag if present
                    origin = self.coord_sys.origin_id
                    if origin in raw_poses:
                        tags_data["0"] = raw_poses[origin]
                    
                    self.jsonreader.write("camera_readings", tags_data)
                
                # Update FPS counter
                frames_count += 1
                elapsed = time.time() - start_time
                if elapsed >= 1.0:  # Update FPS every second
                    with self.lock:
                        self.fps = frames_count / elapsed
                    frames_count = 0
                    start_time = time.time()
                    
            except Exception as e:
                print(f"Error in marker detection thread: {e}")
                time.sleep(0.1)  # Sleep to prevent rapid error loops
    
    def stop(self):
        """Stop the thread"""
        self.running = False
    
    def set_draw_cubes(self, draw_cubes):
        """Set whether to draw cubes on markers"""
        with self.lock:
            self.draw_cubes = draw_cubes
    
    def set_allowed_tags(self, allowed_tags):
        """Set allowed tags filter"""
        with self.lock:
            self.allowed_tags = allowed_tags
    
    def get_latest_results(self):
        """Get the latest detection results"""
        with self.lock:
            return {
                'image': self.latest_image.copy() if self.latest_image is not None else None,
                'transformed': self.latest_transformed.copy(),
                'raw': self.latest_raw.copy(),
                'timestamp': self.latest_timestamp,
                'fps': self.fps,
                'processing_time': self.processing_time
            }
