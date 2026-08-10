#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["stop_integration", "ocr", "ocr.utils"],
    package_dir={
        "stop_integration": "src/stop_integration",
        "ocr": "scripts/ocr",
        "ocr.utils": "scripts/ocr/utils",
    },
)
setup(**d)
