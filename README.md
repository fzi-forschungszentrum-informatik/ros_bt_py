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

All of this (and more) is explained in my Master Thesis "Distributed
Execution of Behavior Trees using Heterogeneous Robot Teams" (link
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

One dependency, `jsonpickle` is not registered in rosdep yet. Please
install it manually, either from the package sources of your distro
(i.e. apt) or via pip.

**Warning:** ros_bt_py is **incompatible** with the latest release of
`jsonpickle`, v1.0.0. Please ensure that you install **v0.9.5**, or
the GUI editor will exhibit strange behavior, and you may not be able
to load saved Behavior Trees.

After installing the dependencies, simply run `catkin_make` and you're
good to go! The command

```bash
$ roslaunch ros_bt_py test.launch
```

will start a BT server and the rosbridge and webserver needed for the
GUI. Afterwards, you can open
http://localhost:8085/ros_bt_py/editor.html to use the editor.

If you just want to monitor the execution of a BT, you might prefer to
use http://localhost:8085/ros_bt_py/viewer.html instead, which gives
you a bigger viewport. This is particularly useful for watching
playback of a rosbag like the one on
`bt_sim_helpers/etc/shove_sim.bag`.

There's even a launchfile in `bt_sim_helpers` to quickly start
playback of that bag:

```
roslaunch bt_sim_helpers playback_shove_test.launch
```
