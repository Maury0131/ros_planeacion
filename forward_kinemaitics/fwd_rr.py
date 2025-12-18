
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
#import sys
from sympy import Matrix, symbols, cos, sin, pi
t=symbols("theta")
d=symbols("d")
a=symbols("a")
aph=symbols("alpha")
L1=0.4
L2=0.3
t1=symbols("theta_1")
t2=symbols("theta_2")

Rz = Matrix([[cos(t),-sin(t),0,0],[sin(t),cos(t),0,0],[0,0,1,0],[0,0,0,1]])
Rx = Matrix([[1,0,0,0],[0,cos(aph),-sin(aph),0],[0,sin(aph),cos(aph),0],[0,0,0,1]])
Tz = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
Tx = Matrix([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
A=Rz*Tz*Tx*Rx
A01=A.subs({t:t1,d:0,a:L1,aph:0})
A12=A.subs({t:t2,d:0,a:L2,aph:0})


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
        j1=msg.position[1]
        j2=msg.position[2]
        #T=A01*A12
        T=A.subs({t:0,d:0,a:0,aph:pi/2})*A01*A12
        res=T.subs({t1:j1,t2:j2})
        print(res)

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
