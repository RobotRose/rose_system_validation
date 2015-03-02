#! /usr/bin/env python
try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

config = {
    'description': 'Get info about wireless LAN and analysis',
    'author': 'Loy van Beek',
    'url': 'TODO',
    'download_url': 'TODO',
    'author_email': 'beek@robot-rose.nl',
    'version': '0.1',
    'install_requires': ['sh'],
    'packages': ['wlan'],
    'scripts': [],
    'name': 'wlan'
}

setup(**config)
