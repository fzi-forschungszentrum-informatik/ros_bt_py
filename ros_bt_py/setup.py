#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    version='0.0.0',
    scripts=[],
    packages=['ros_bt_py',
              'ros_bt_py.nodes',
              'ros_bt_py.nodes.migrations',
              'ros_bt_py.ros_nodes',
              'ros_bt_py.ros_nodes.migrations'],
    package_dir={'': 'src'},
    install_requires=[
        'requests'
    ]
)

setup(**setup_args)
