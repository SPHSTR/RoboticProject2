import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3 

class VelocityControlNode(Node):
    def __init__(self):
        super().__init__('velocity_control_node')
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Vector3 , 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Publisher
        self.cnt_vel_pub = self.create_publisher(Vector3 , 'cnt_vel', 10)


    def cmd_vel_callback(self, msg:Vector3):
        self.compute_control(msg)


    def compute_control(self, msg:Vector3):
        # m/s   
        X = msg.x
        Y = msg.y
        theta= msg.z


        # M1_setpoint = (-Y + (-theta * 0.100 * 3)) 
        # M2_setpoint = ((Y * math.sin(math.pi/6)) + (X * math.cos(math.pi/6)) + (-theta * 0.100 * 3))
        # M3_setpoint = ((Y * math.sin(math.pi/6)) - (X * math.cos(math.pi/6)) + (-theta * 0.100 * 3))

        control_output = Vector3()
        control_output.x = X
        control_output.y = Y
        control_output.z = theta

        self.cnt_vel_pub.publish(control_output)
        print(f"Publishing cnt_vel: M1={control_output.x}, M2={control_output.y}, M3={control_output.z}")
        # self.get_logger().info(f'cnt_vel: {control_output.data}')



def main(args=None):
    rclpy.init(args=args)
    node = VelocityControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()