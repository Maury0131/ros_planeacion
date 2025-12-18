

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
#import sys
from sympy import Matrix, symbols, cos, sin, pi
t=symbols("theta")
p0=Matrix([[0.4],[0],[0]])
          
Rx= Matrix([
    [1, 0, 0],
    [0, cos(t), -sin(t)],
    [0, sin(t), cos(t)]
])

# Matriz de rotación en el eje Y
Ry = Matrix([
    [cos(t), 0, sin(t)],
    [0, 1, 0],
    [-sin(t), 0, cos(t)]
])

# Matriz de rotación en el eje Z
Rz = Matrix([
    [cos(t), -sin(t), 0],
    [sin(t), cos(t), 0],
    [0, 0, 1]
])



joint_names=['joint1','joint1_1','joint2']

class JointSubscriber(Node):

    def __init__(self):
        super().__init__('joint_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.sub_callback,
            10
        )
        self.subscription  # evita warning de variable sin usar

    def sub_callback(self, msg):
        #print(msg.position[0], msg.position[1], msg.position[2])
        t1=msg.position[0]
        t2=msg.position[1]
        p1=Rz.subs(t,t1)*(Ry.subs(t,-t2)*p0)
        print(p1)

def main(args=None):
    rclpy.init(args=args)

    joints_sub = JointSubscriber()

    rclpy.spin(joints_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #rclpy.spin_once(joints_pub, timeout_sec=1.0)
    
    #joints_pub.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
