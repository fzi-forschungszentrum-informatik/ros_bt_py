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
