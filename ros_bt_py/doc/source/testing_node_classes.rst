.. _testing-nodes:

Testing Node Classes
====================

Even though a lot of errors are harder to make and/or easier to catch
with :mod:`ros_bt_py` than with other libraries, you should still
test any new node classes you make.

The easiest way to to this is by adding tests to your ROS package.
To do that, you simply create a new folder, slap some test scripts in there, and add a few lines to your :code:`CMakeLists.txt` and :code:`package.xml`.
I'll walk you through it:

Create a Test Folder and Test Script
------------------------------------

The most common way to write Python unit tests is the built-in
:mod:`unittest` module. It's not too fancy, but it's guaranteed to be
available and has everything you need for moderate-sized tests.

So, within your ROS package, create a new folder called
:code:`test/unittest`.  You don't technically need the
:code:`unittest` folder, but if you do eventually need to add some
rostest_ tests, it's just nicer to have :code:`test/unittest` and
:code:`test/rostest`.

.. _rostest: http://wiki.ros.org/rostest

Next, create a new file called :code:`test_my_node.py` and put the following code in::

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

A lot of the names here are important, because ROS uses nose_ to find and run tests.
Your test script must start with the word "test".
The same goes for the class inside it and the test methods inside that class.

And yes, even though camel-case is not usually preferred in ROS code,
use it for test cases, because nose_ works that way - the
:mod:`unittest` module does, too, by the way, as you can see from the
:meth:`assertEqual()` call above.

.. _nose: http://nose.readthedocs.io
