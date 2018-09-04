"""
Allows for develspace use of `pheeno_ros` python package.
Do NOT run using python. Let catkin do it.
More info: http://docs.ros.org/api/catkin/html/user_guide/setup_dot_py.html

Written by: Zahi Kakish (zmk5)

"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

SETUP_ARGS = generate_distutils_setup(
    packages=["pheeno_ros"],
    package_dir={'': 'scripts/'}
)

setup(**SETUP_ARGS)
