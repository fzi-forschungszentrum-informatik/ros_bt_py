[![pipeline status](/../badges/master/pipeline.svg)](/) [![coverage](/../badges/master/coverage.svg)](/)

# Welcome to ros_bt_py!

This is a [Behavior
Tree](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control))
library meant to be an alternative to
[SMACH](http://wiki.ros.org/smach),
[FlexBE](http://wiki.ros.org/flexbe) and the like.

It includes a ReactJS-based web GUI and all the building blocks you
need to build moderately advanced mission control Behavior Trees
without writing a single line of code!

Its claim to fame is the ability to *shove*, or transparently
remote-execute, parts of a Behavior Tree.

## Documentation

All of this (and more) is explained in the Master Thesis "Distributed
Execution of Behavior Trees using Heterogeneous Robot Teams" by Nils Berg (link
coming as soon as I'm certain where I can post the PDF).

There is also a growing amount of Sphinx documentation in the `doc`
folder - simply execute the following commands in your shell to get
browsable HTML documentation, including some tutorials:

```bash
$ cd ros_bt_py/doc
$ make html
$ cd build
$ python -m SimpleHTTPServer & xdg-open http://localhost:8000/html
```

## Installation

To actually start using ros_bt_py, you need to install its dependencies first:

```bash
$ cd catkin_workspace
$ rosdep install --from-paths src --ignore-src -r -y
```

One dependency, `jsonpickle` might not be registered in rosdep yet. Please
install it manually, either from the package sources of your distro
(i.e. apt) or via pip.

**Warning**
PyYaml 5.x breaks interoperability with saved trees, make sure you have PyYaml <= 3.13 installed!
```
pip install 'pyyaml<=3.13'
```

After installing the dependencies, simply run `catkin_make` and you're
good to go! The command

```bash
$ roslaunch ros_bt_py ros_bt_py.launch
```

will start a BT server and the rosbridge and webserver needed for the
GUI. Afterwards, you can open
http://localhost:8085/ros_bt_py/editor.html to use the editor.
