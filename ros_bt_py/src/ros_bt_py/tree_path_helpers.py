#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
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
