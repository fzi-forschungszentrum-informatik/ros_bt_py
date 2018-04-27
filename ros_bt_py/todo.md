- Figure out (web-compatible) serialization

  - NodeData? (how to represent types / values for non-basic types?)

      One idea: Publish map Typename => JSON structure

  - Connections? Are direct connections enough, or too cumbersome?

      Also: add auto_wire() method?

- add_child needs to be in Node (generate child IDs and set their parent property)

- traverse tree to find child with given ID

- IDs are just child indices concatenated with dots in between:

    Root: 0

    Root->Sequence->First Child(Sequence)->Second Child: 0.0.0.1

- IDs can be local if they start with a dot:

    Current node's ID: 0.1.3

    Relative ID .4 => 0.1.3.4

- Maybe use names / labels instead of indices?

    In that case, change name to oldNameN for Nth node with same name

- Publish available Nodes

    BehaviorTreeServer node needs parameter that tells it what
    packages to load

    Maybe give nodes `__init__.py` a string constant or sthg? Star
    imports are still icky...

    Or: Scan modules / submodules for subclasses of Node

# Implementation of Subtrees
## Action interface
The lifetime of the action implicitly defines the lifetime of the tree on the server.

Two options:

1. Action call means "run this tree (with these - unchanging - inputs)
   until it succeeds"

   Feedback contains tree state after each tick

2. Action call means "load this tree"

   Feedback returns the ROS namespace for the services to control this
   tree (potential leaking of Service IDs over time? roscore might not
   like that)

## Subtree Node
Needs to wrap Action Interface

## Local / Remote execution
Implemented via decorator.

The subtree node itself has an input (or option? decide whether
options should really be static) for action server address.

Decorator decides what server to use (remote, local, which remote?) &
sets input of subtree node.

Caveats:

* Decorator cannot switch server while subtree is running
* Add subtree groups that cannot be executed on different robots?
  (e.g. "walk to target" & "pick up target")
