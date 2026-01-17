from setuptools import find_packages
from setuptools import setup

setup(
    name='mav_planning_msgs',
    version='3.3.3',
    packages=find_packages(
        include=('mav_planning_msgs', 'mav_planning_msgs.*')),
)
