#!/usr/bin/env python3
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    name="line_follow_integration",
    version="0.1.0",
    packages=["line_follow_integration"],
    package_dir={"": "src"},
)

setup(**setup_args)
