.. _testing-nodes:

Testing Node Classes
====================

Even though a lot of errors are harder to make and/or easier to catch
with :mod:`ros_bt_py` than with other libraries, you should still
test any new node classes you make.

The easiest way to to this is by adding tests to your ROS package.  To
do that, you simply create a new folder, slap some test scripts in
there, and add a few lines to your :code:`CMakeLists.txt` and
:code:`package.xml`.  I'll walk you through it:

Creating a Test Folder and Unit Test Script
-------------------------------------------

The most common way to write Python unit tests is the built-in
:mod:`unittest` module. It's not too fancy, but it's guaranteed to be
available and has everything you need for moderate-sized tests.

So, within your ROS package, create a new folder called
:code:`test/unittest`.  You don't technically need the
:code:`unittest` folder, but if you do eventually need to add some
rostest_ tests, it's just nicer to have :code:`test/unittest` and
:code:`test/rostest`.

Next, create a new file called :code:`test_my_node.py` and put the
following code in it::

  import unittest

  # Useful to have the state names to compare against
  from ros_bt_py_msgs.msg import Node as NodeMsg

  # REALLY useful node class that counts how many times it's been
  # ticked, unticked, reset etc.
  #
  # It also lets you customize what states it returns when ticked,
  # using an Option
  from ros_bt_py.nodes.mock_nodes import MockLeaf

  from my_pacakge.nodes.my_awesome_node import MyAwesomeNode

  class TestMyAwesomeNode(unittest.TestCase):
      def testSetup(self):
          awesome = MyAwesomeNode()
          awesome.setup()

          # Any node should be IDLE after calling setup()
          self.assertEqual(awesome.state, NodeMsg.IDLE)

A lot of the names here are important, because ROS uses nose_ to find
and run tests.  Your test script must start with the word "test".  The
same goes for the class inside it and the test methods inside that
class.

And yes, even though camel-case is not usually preferred in ROS code,
use it for test cases, because nose_ works that way - the
:mod:`unittest` module does, too, by the way, as you can see from the
:meth:`assertEqual()` call above.

.. _nose: http://nose.readthedocs.io

Running your Unit Tests
-----------------------

Now, to actually *run* your unit tests, simply go to the test folder and run `nosetests`::
  .. highlight:: bash
  $ nosetests --no-byte-compile

The `--no-byte-compile` option avoids cluttering your workspace with
`.pyc` files. That's useful because it just *looks* nicer, but also
because you're almost certain to run into some sort of weird bug where
you change your code, but the results of your tests stay the same,
because Python sees and uses an outdated `.pyc` file. So, better not
to create them in the first place.

Nose automatically runs all the tests it finds.
Whenever you add a new test case, please make sure it is actually detected::
  .. highlight:: bash
  $ nosetests --no-byte-compile -v | grep testMyAwesomeNode

`testMyAwesomeNode`, of course, being the name of your test case.

Adding your Unit Tests to CMakeLists.txt
----------------------------------------

If you want your tests to be run by the CI pipeline (and who
doesn't?), you'll also have to tell the build system about them.

Don't worry, it's easy - just add the following lines to your
`CMakeLists.txt`, and nose will do the rest::
  .. highlight:: cmake
  if (CATKIN_ENABLE_TESTING)
    ## Add folders to be run by python nosetests
    # nosetest automatically collects test methods, no need for main
    # methods in your test files!
    catkin_add_nosetests(test/unittest)
  endif()

Creating a rostest
------------------

There are situations where unit tests simply aren't enough.  In most
cases, that's because you need a ROS core running to use some ROS
resources.

For these cases, there's rostest_.

Rostests consist of two major components:

1. The `.test` File

  This uses the same syntax as a `.launch` file, which should be
  familiar. As an added bonus, all executable Python scripts in the
  same folder as the `.test` file are available to launch via the
  `<node>` tag. You can use this to add mock nodes for your tests to
  interact with, as shown in the example below.

  The other difference to a regular `.launch` file is the `<test>`
  tag. The rostest_ documentation explains this in more detail, but
  essentially, each test is run isolated from all the others, in a
  fresh environment. Again, see the example below (from this package)
  for a look at how that tag works.

  .. literalinclude:: ../../test/rostest/ros_leaves.test
     :language: xml

2. The Test Code

  rostest_ test files use `unittest`, just like regular unit tests,
  but unlike those, they need to be executable, so they need a `#!`
  line at the beginning, as well as a proper `main` method.

  Here's an example::
    :linenos:
      if __name__ == '__main__':
	  rospy.init_node('test_topic_subscribe_leaf')
	  import rostest
	  import sys
	  import os
	  os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_topic_subscribe_leaf')
	  rostest.rosrun(PKG, 'test_topic_subscribe_leaf', TestTopicSubscriberLeaf,
			 sysargs=sys.argv + ['--cov'])

  If you don't want coverage data, you can leave out lines 4-6 and 8.

Running a rostest
-----------------

Running a rostest is just as easy as running a `.launch` file.
Simply run::
  .. highlight:: bash
  $ rostest my_rostest.test

Adding a rostest to CMakeLists.txt
----------------------------------

Again, it's not hard to add a rostest to `CMakeLists.txt`::
  .. highlight:: cmake
  if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/rostest/my_rostest.test)
  endif()

However, you'll also need to add the following line to `package.xml`::
  .. highlight:: xml
  <build_depend>rostest</build_depend>


Running Tests via catkin
------------------------

Now that you've registered your tests with catkin, you can use catkin
to run them::
  .. highlight:: bash
  $ catkin_make run_tests

Note that there **is** a `test` target for catkin. That target **does
not** run your tests!

.. _rostest: http://wiki.ros.org/rostest
