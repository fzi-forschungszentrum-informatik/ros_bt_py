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
from collections import OrderedDict

from ros_bt_py.migration import Migration, migration
from ros_bt_py.node_config import OptionRef
from ros_bt_py.nodes.sequence import Sequence


class NodeWithoutMigrationFunction(Migration):
    pass


class NodeWithoutMigrationToFirstVersion(Migration):
    @migration(from_version='1.0.0', to_version='1.1.0')
    def first_migration(self):
        pass


class NodeWithoutVersionAndWithFirstMigration(Migration):
    @migration(from_version='', to_version='1.0.0')
    def first_migration(self):
        pass


class NodeWithBrokenMigrationPath(Migration):
    @migration(from_version='', to_version='0.8.0')
    def first_migration(self):
        pass

    @migration(from_version='0.9.0', to_version='1.0.0')
    def broken_migration(self):
        pass


class NodeWithIdenticalVersions(Migration):
    @migration(from_version='', to_version='0.8.0')
    def first_migration(self):
        pass

    @migration(from_version='0.8.0', to_version='0.8.0')
    def broken_migration(self):
        pass


class NodeWithWorkingMigrations(Migration):
    @migration(from_version='', to_version='0.8.0')
    def first_migration(self):
        # call the log functions
        self.logdebug('available')
        self.loginfo('available')
        self.logwarn('available')
        self.logerr('available')
        self.logfatal('available')

    @migration(from_version='0.8.0', to_version='1.0.0')
    def second_migration(self):
        self.add_option('valid_option2', int, self.get_option('valid_option'))
        self.add_option('type_option', type)
        self.add_input('option_ref', OptionRef('type_option'))
        self.add_output('valid_output2', int)
        self.rename_option('valid_option2', 'valid_option3')
        self.remove_option('valid_option3')

        self.add_option('option_type', type)
        self.add_option('option_int', int)
        self.add_option('option_str', str)
        self.add_option('option_float', float)
        self.add_option('option_bool', bool)
        self.add_option('option_list', list)
        self.add_option('option_dict', dict)
        self.add_option('option_ordereddict', OrderedDict)
        self.add_option('option_object', object)


class NodeWithWorkingMigrationsChangeType(Migration):
    @migration(from_version='', to_version='1.0.0')
    def first_migration(self):
        self.change_option_type('change_type', str, 'hi')
        self.change_input_type('change_type', str)
        self.change_output_type('change_type', str)


class NodeWithWorkingMigrationsThatChangesTree(Migration):
    @migration(from_version='', to_version='0.8.0')
    def first_migration(self):
        pass

    @migration(from_version='0.8.0', to_version='1.0.0')
    def add_sequence(self):
        self.get_parent_node()
        seq = Sequence()
        seq.add_child(self.make_node(name=self.get_name()))

        self.add_node(seq)
        self.add_node(Sequence())
        self.add_node_msg(Sequence(name='unique').to_msg())

        self.replace_at_parent(seq.name)

        self.change_node_class('NewNodeClass')
        self.change_module('new_module')


class NodeWithGetOptionException(Migration):
    @migration(from_version='', to_version='1.0.0')
    def first_migration(self):
        self.get_option('does_not_exist')


class NodeWithAddOptionException(Migration):
    @migration(from_version='', to_version='1.0.0')
    def first_migration(self):
        self.add_option('valid_option', int, 0)


class NodeWithAddInputException(Migration):
    @migration(from_version='', to_version='1.0.0')
    def first_migration(self):
        self.add_input('valid_input', int)
        self.add_input('valid_input', int)


class NodeWithAddOutputException(Migration):
    @migration(from_version='', to_version='1.0.0')
    def first_migration(self):
        self.add_output('valid_output', int)


class NodeWithOptionrefException(Migration):
    @migration(from_version='', to_version='1.0.0')
    def first_migration(self):
        self.add_output('broken', OptionRef('does_not_exist'))
