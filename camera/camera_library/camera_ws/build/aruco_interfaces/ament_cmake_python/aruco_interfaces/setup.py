from setuptools import find_packages
from setuptools import setup

setup(
    name='aruco_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('aruco_interfaces', 'aruco_interfaces.*')),
)
