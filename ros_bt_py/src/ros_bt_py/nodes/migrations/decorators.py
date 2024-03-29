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
from ros_bt_py.node_config import OptionRef


class IgnoreFailure(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class IgnoreRunning(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class IgnoreSuccess(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class UntilSuccess(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class Inverter(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class Retry(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class Repeat(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class RepeatNoAutoReset(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class RepeatAlways(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class RepeatUntilFail(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class RepeatIfFail(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class Throttle(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class ThrottleSuccess(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class Optional(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass


class Watch(Migration):
    @migration(from_version="", to_version="0.9.0", changelog="adding version number")
    def adding_version(self):
        pass
