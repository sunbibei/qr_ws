#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['qr_panel', 'qr_panel.data_plot'],
    package_dir={'': 'src'},
    scripts=['scripts/qr_panel']
)

setup(**d)
