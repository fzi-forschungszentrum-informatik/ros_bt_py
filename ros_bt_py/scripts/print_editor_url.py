#! /usr/bin/env python2.7

import rospy


def main():
    rospy.init_node('print_editor_url', anonymous=True)

    port = rospy.get_param('~port', default=8085)

    info_str = '# ros_bt_py editor available at http://localhost:'
    info_str += str(port) + '/ros_bt_py/editor.html #'
    padding_str = '{:#>{w}}'.format("", w=len(info_str))

    print('\n' + padding_str + '\n' + info_str + '\n' + padding_str + '\n')
    return 0

if __name__ == '__main__':
    main()
