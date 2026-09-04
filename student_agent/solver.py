"""
Write your own solver in the scan_callback function
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ==========================================
# These four parameters MUST add up to exactly 30!
# ==========================================
TOP_SPEED = 7
ACCELARATION = 12
TURN_SPEED = 8
SENSOR_RANGE = 3



class StudentSolver(Node):
    def __init__(self):
        super().__init__('student_solver')
        
        # subscriber to read sensor values (L,F,R)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/mouse/scan',
            self.scan_callback,
            10
        )
        
        # publisher to send movement commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mouse/cmd_vel',
            10
        )
        
        self.get_logger().info("Student Solver Node initialized successfully.")
        self.get_logger().info(f"Stats -> Speed: {TOP_SPEED}, Accel: {ACCELARATION}, Turn: {TURN_SPEED}, Range: {SENSOR_RANGE}")



    def scan_callback(self, msg):
        """
        This function runs every time a new sensor reading is received (at 20 Hz).
        msg.ranges contains the distances:
        msg.ranges[0] -> Left ray distance
        msg.ranges[1] -> Front ray distance
        msg.ranges[2] -> Right ray distance
        """
        d_left = msg.ranges[0]
        d_front = msg.ranges[1]
        d_right = msg.ranges[2]
        
        cmd = Twist()
        """
        Wall Following algorithm

        prop = 3.0
        target = 0.5
        error = target - d_right

        if d_front < 0.6:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5
        elif d_right > 0.55:
            cmd.linear.x = 0.5
            cmd.angular.z = prop * error
        elif d_right < 0.45:
            cmd.linear.x = 0.5                
            cmd.angular.z = prop * error
        else:
            cmd.linear.x = 3.0
            cmd.angular.z = 0.0
        """
            
        self.cmd_pub.publish(cmd)
    


        

def main(args=None):
    rclpy.init(args=args)
    node = StudentSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()