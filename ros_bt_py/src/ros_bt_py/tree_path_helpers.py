CONTROL_CHARACTERS = {
    '/': 'path_separator',
    '[': 'index_start',
    ']': 'index_end'
}


def increment_child_name(name):
    """If `name` ends in a number (of the shape '_$number'), increment that number.

    Otherwise, add '_2'.

    :param basestring name: The name to increment

    :rtype: basestring
    :returns: A new string according to the above
    """
    # Regex match for _([0-9]+)

    # If match: replace with _($1 + 1)

    # If not: Append _2 to name

    # return
    pass


def add_to_path(path, name):
    """Adds `name` to `path`, ensuring the resulting string is a valid path.

    :param basestring path:
    :param basestring name:

    :rtype: basestring
    """
    pass


def pathify(path):
    """
    Escape the Tree path control characters (see `CONTROL_CHARACTERS`)
    """
    pass
