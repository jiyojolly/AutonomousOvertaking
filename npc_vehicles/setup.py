"""
Setup for npc_vehicles
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['npc_vehicles'],
    package_dir={'': 'src'}
)

setup(**d)
