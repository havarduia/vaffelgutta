from setuptools import find_packages
from setuptools import setup

setup(
    name='aruco_pose_estimation',
    version='2.0.0',
    packages=find_packages(
        include=('aruco_pose_estimation', 'aruco_pose_estimation.*')),
)
