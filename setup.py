#! /usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rose_system_validation'],
    # scripts=['wlan/wlan.py'],
    package_dir={'': 'src'},
    install_requires=['sh']
)

setup(**d)