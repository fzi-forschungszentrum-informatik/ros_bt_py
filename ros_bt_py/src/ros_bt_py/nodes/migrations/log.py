from ros_bt_py.migration import Migration, migration
from ros_bt_py.node_config import OptionRef
from ros_bt_py.ros_helpers import LoggerLevel


class Log(Migration):
    @migration(from_version='', to_version='0.9.0', changelog='adding version number')
    def adding_version(self):
        pass

    @migration(
        from_version='0.9.0', to_version='1.0.0',
        changelog='change logger_level type to enable a dropdown menu in the editor')
    def change_logger_level_type(self):
        logger_level = self.get_option(key='logger_level')

        # if logger_level is foo change it to info
        if logger_level == 'foo':
            logger_level = 'info'

        if logger_level == 'warn':
            logger_level = 'warning'

        if logger_level == 'err':
            logger_level = 'error'

        self.remove_option(key='logger_level')
        self.add_option(
            key='logger_level',
            data_type=LoggerLevel,
            initial_value=LoggerLevel(logger_level=logger_level))

    @migration(
        from_version='1.0.0', to_version='2.0.0',
        changelog='add log_type option to allow logging of arbitrary types')
    def add_log_type_option(self):
        self.add_option(
            key='log_type',
            data_type=type,
            initial_value=str)
        self.change_input_type(
            key='in',
            data_type=OptionRef('log_type'))
