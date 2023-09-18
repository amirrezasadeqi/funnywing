#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['wing_modules', 'RfCommunication'],
    package_dir={'wing_modules': 'wing_modules',
                 'RfCommunication': 'RfCommunication'}
)

setup(**setup_args)
