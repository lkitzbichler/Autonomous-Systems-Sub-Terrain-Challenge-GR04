from setuptools import find_packages
from setuptools import setup

setup(
    name='mav_msgs',
    version='3.3.3',
    packages=find_packages(
        include=('mav_msgs', 'mav_msgs.*')),
)
