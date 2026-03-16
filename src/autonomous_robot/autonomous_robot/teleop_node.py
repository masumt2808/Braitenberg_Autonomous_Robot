import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

KEYS = {
    'i': ( 0.5,  0.0, 'FORWARD'),
    ',': (-0.5,  0.0, 'BACKWARD'),
    'j': ( 0.0,  1.0, 'LEFT'),
    'l': ( 0.0, -1.0, 'RIGHT'),
    'k': ( 0.0,  0.0, 'STOP'),
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Teleop: i=fwd  ,=back  j=left  l=right  k=stop  q=quit')

    def run(self):
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                key = sys.stdin.read(1)
                if key == 'q': break
                if key in KEYS:
                    lx, az, cmd = KEYS[key]
                    t = Twist()
                    t.linear.x  = float(lx)
                    t.angular.z = float(az)
                    self.pub.publish(t)
                    print(f'\r[{cmd}] linear={lx} angular={az}')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self.pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
