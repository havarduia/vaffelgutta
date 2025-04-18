"""
Coordinate transformation functionality.
"""

import numpy as np
from camera.config.configloader import ConfigLoader


class CoordinateSystem:
    """
    Class for transforming coordinates between different reference frames.
    """

    def __init__(self):
        """Initialize the coordinate transformer."""
        self.config_loader = ConfigLoader()

        # Coordinate system setup
        self.origin_id = self.config_loader.get("origin_id")
        self.offset_x = self.config_loader.get("offset_x")
        self.offset_y = self.config_loader.get("offset_y")
        self.offset_z = self.config_loader.get("offset_z")
        self.bias_x = self.config_loader.get("bias_x")
        self.bias_y = self.config_loader.get("bias_y")
        self.bias_z = self.config_loader.get("bias_z")
        self.last_origin_inv = (
            None  # Store last known inverse transformation from origin
        )

    def transform_poses(self, poses):
        """
        Transform poses from camera frame to robot frame using the origin marker.

        Args:
            poses: Dictionary mapping marker IDs to poses in camera frame

        Returns:
            dict: Dictionary mapping marker IDs to poses in robot frame
        """
        if not poses:
            return {}

        # Check if origin marker is detected
        if self.origin_id in poses:
            origin_inv = np.linalg.inv(poses[self.origin_id])
            self.last_origin_inv = origin_inv
        elif self.last_origin_inv is None:
            # No origin marker and no previous transformation
            return {}
        else:
            # Use last known transformation
            origin_inv = self.last_origin_inv

        transformed_poses = {}
        for id, pose in poses.items():
            # Transform from camera frame to origin frame
            o_to_t = origin_inv @ np.array(pose)

            # Apply coordinate system transformation and offsets
            transformed_matrix = np.array(
                [
                    [
                        o_to_t[1, 1],
                        -o_to_t[1, 0],
                        o_to_t[1, 2],
                        o_to_t[1, 3] + self.bias_x - self.offset_x,
                    ],
                    [
                        -o_to_t[0, 1],
                        o_to_t[0, 0],
                        -o_to_t[0, 2],
                        -o_to_t[0, 3] + self.bias_y - self.offset_y,
                    ],
                    [
                        o_to_t[2, 1],
                        -o_to_t[2, 0],
                        o_to_t[2, 2],
                        o_to_t[2, 3] + self.bias_z - self.offset_z,
                    ],
                    [0, 0, 0, 1],
                ]
            ).tolist()

            transformed_poses[id] = transformed_matrix

        return transformed_poses

    def get_origin_transform(self):
        """
        Get the current origin transformation matrix.

        Returns:
            numpy.ndarray: The inverse of the origin transformation matrix
        """
        return self.last_origin_inv
