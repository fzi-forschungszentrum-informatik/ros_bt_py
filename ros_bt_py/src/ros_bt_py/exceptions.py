class BehaviorTreeException(Exception):
    pass

class NodeConfigError(BehaviorTreeException):
    pass

class NodeStateError(BehaviorTreeException):
    pass
