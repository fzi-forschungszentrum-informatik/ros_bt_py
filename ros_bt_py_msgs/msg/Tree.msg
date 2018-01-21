# A descriptive name for this tree. At runtime, this is chosen by the
# tree manager node.
string name
# A string pointing to the YAML file this tree was read from, if
# any. Can be a file:// or a package:// string.
string path
Node[] nodes
NodeDataWiring[] data_wirings
# One of the constants below
string state

string IDLE = IDLE
string EDITING = EDITING
string TICKING = TICKING
string DEBUGGING = DEBUGGING

# These are accessible when using this tree as a subtree
NodeDataLocation[] public_node_data
