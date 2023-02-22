# Copyright 2018-2023 FZI Forschungszentrum Informatik
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
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
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


from ros_bt_py.migration import Migration, migration
from ros_bt_py.ros_helpers import LoggerLevel


class IterateList(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="added version")
    def adding_version(self):
        pass

    @migration(
        from_version="0.9.0",
        to_version="0.9.1",
        changelog="Running once without ticking child",
    )
    def adding_version_091(self):
        pass

    @migration(
        from_version="0.9.1", to_version="0.9.2", changelog="Failing on failed child"
    )
    def adding_version_092(self):
        pass

    @migration(
        from_version="0.9.2", to_version="1.0.0", changelog="Succeed on empty list"
    )
    def adding_version_100(self):
        pass


class IsInList(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="added version")
    def adding_version(self):
        pass


class InsertInList(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="added version")
    def adding_version(self):
        pass


class GetListElementOption(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="added version")
    def adding_version(self):
        pass


class ListLength(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="added version")
    def adding_version(self):
        pass
