# ros\_bt\_py

## general idea

- Every node has tick() and untick() methods (untick tells a node that
  a new tick cycle has begun, but it **hasn't** been ticked, so it
  should stop whatever it is currently doing.
- every node class needs to be decorated with BTNode()
  - BTNode() takes three dicts (each containing string : type mappings):
    1. options:

        the class will take a dict of those at build time (i.e. in its
        constructor) and type check them. It will generate getters
        so that they can be used as inputs for children.

    2. inputs:

        a \_setter\_ will be generated for each. every setter *needs* to
        be called at least once before calling tick(), **with an
        object of the correct type**!

        Additional convenience function: assignSource(getter,
        set_input). Will attempt to automatically call getter when
        tick() is called

    3. outputs / traits (not sure about the final name yet):

        generate a getter for each

    All getters / setters have PEP 484 type annotations / comments
        (comments for now, annotations when I'm sure ROS supports
        python 3.5)

    Option and output getters are public and can be called
    directly. The input setters shouldn't be called directly, but
    instead used via assignSource()


## Scope

- This is python, so everything's wide open anyway.
- Nothing leaks upwards or downwards automatically (no automatic
  inputs, everything needs to be assigned!)
- Outputs can be connected to inputs of siblings, but!
  - At construction time, each node will check if there's possible
    conflicts among its children:
    1. if any child input hasn't been assigned a getter, that's a **fatal** error
    2. if a child input has been assigned a getter from a child that
       may execute after it (or never), that's a **warning**
    3. any unconnected outputs are logged for possible **debugging**

## Tree Construction

- Start at leaf nodes
- Check consistency of each node before moving on to its parent
