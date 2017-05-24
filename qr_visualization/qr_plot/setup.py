#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['qr_plot', 'qr_plot.rqt_plot_lf'],
    package_dir={'': 'src'},
    scripts=['scripts/qr_plot']
)

setup(**d)
