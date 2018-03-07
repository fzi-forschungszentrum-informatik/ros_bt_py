# If this is true, subtree_path needs to be set. If it is false,
# package and class need to be set instead.
bool is_subtree

# Can be one of 3 things:
# 1. a file:// URL pointing directly to a YAML file
# 2. a package:// URL pointing to a YAML file
# 3. a string that contains YAML representing a subtree
string subtree_data
# The python module the node is from
string module
string node_class

# The maximum amount of children this node supports, where -1 means
# infinite children
int32 max_children

## These are run-time parameters
# Each node name must be unique in its tree. This does not count for
# subtrees, as they're handled separately.
string name
# Order is important for child_names!
string[] child_names
NodeData[] options
NodeData[] inputs
NodeData[] outputs
# Should be one of the constants below
string state

string UNINITIALIZED = UNINITIALIZED
string IDLE = IDLE
string RUNNING = RUNNING
string SUCCEEDED = SUCCEEDED
string SUCCEED = SUCCEEDED
string SUCCESS = SUCCEEDED
string FAILED = FAILED
string FAIL = FAILED
string FAILURE = FAILED
string BROKEN = BROKEN
string PAUSED = PAUSED
string SHUTDOWN = SHUTDOWN
string DEBUG_PRE_TICK = DEBUG_PRE_TICK
string DEBUG_TICK = DEBUG_TICK
string DEBUG_POST_TICK = DEBUG_POST_TICK
