class BehaviorTreeException(Exception):
    pass


class NodeConfigError(BehaviorTreeException):
    pass


class NodeStateError(BehaviorTreeException):
    pass


class TreeTopologyError(BehaviorTreeException):
    pass


class MissingParentError(BehaviorTreeException):
    pass
