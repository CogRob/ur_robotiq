#!/usr/bin/env python
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        'ur5e_2f85'],
    package_dir={'': 'src'},
)

setup(**setup_args)
