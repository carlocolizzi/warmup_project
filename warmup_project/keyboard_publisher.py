import threading
import rclpy
import os

from simple_message.msg import SimpleMessage

NODE_NAME = "simple_publisher"


def handle_keyboard(publisher):
    while True:
        print('\n- Simple Publisher Menu -')
        print('   1. Command (Move along path 1)')
        print('   2. Command (Move along path 2)')
        print('   99. Exit')

        menu = input('Input the menu: ')

        if menu == '1':
            msg = SimpleMessage()
            msg.command_id = SimpleMessage.COMMAND_FOR_DEMO_1
            publisher.publish(msg)
            print(" \n>>> command is published : {0}".format(msg.command_id))

        elif menu == '2':
            msg = SimpleMessage()
            msg.command_id = SimpleMessage.COMMAND_FOR_DEMO_2
            publisher.publish(msg)
            print(" \n>>> command is published : {0}".format(msg.command_id))

        elif menu == '99':
            rclpy.shutdown()
            os._exit(1)


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(NODE_NAME)
    publisher = node.create_publisher(SimpleMessage, SimpleMessage.NAME)

    th = threading.Thread(target=handle_keyboard, args=(publisher,))
    th.start()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()