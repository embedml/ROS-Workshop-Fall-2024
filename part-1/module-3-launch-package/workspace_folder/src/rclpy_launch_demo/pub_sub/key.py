import rclpy      # ROS CLIENT LIBRARIES
from rclpy.node import Node

from std_msgs.msg import String

class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard')
        self.publisher_ = self.create_publisher(String, 'key_press', 10)
        # listener.start()  # start to listen on a separate thread
        # listener.join() 
        while self._context.ok():
            key = input("command:")
            self.on_press(key)

    def on_press(self, key):
        if key in ['a', 's', 'd', 'w' ]:  # keys of interest
            # self.keys.append(k)  # store it in global-like variable
            msg = String()
            msg.data = key
            self.publisher_.publish(msg)
        elif key == 'EML CLUB!':
            print ("THAT'S RIGHT")
        else:
            print ('nope: %s' % key)


def main(args=None):
    rclpy.init(args=args)

    key_publisher = KeyboardPublisher()

    rclpy.spin(key_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    key_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
