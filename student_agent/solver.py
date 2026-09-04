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

size = 16    #Size of maze

wall_vert = [[0 for _ in range(size + 1)] for _ in range(size)]        #1 in current cell indicates wall to the west, 1 in cell to the right indicates wall to the east
wall_horz = [[0 for _ in range(size)] for _ in range(size + 1)]        #1 in current cell indicates wall to the north, 1 in cell below indicates wall to the south
flood = [[0 for _ in range(size)] for _ in range(size)]            #Contains flood values

directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]         #Helps compare cells adjacent to a given cell
#              N        E         S      W


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
        
        #ADJUST THESE ACCORDINGLY
        self.turn_time = 1.59     #Time to turn 90 degrees
        self.cell_time = 1.0     #Time to cross one cell distance
        self.settle_time = 1.0
        self.wall_thresh = 0.8   #Sensor reading below this indicates a wall
        self.goals = [(7, 7), (7, 8), (8, 7), (8, 8)]         #Indices of goal cells
        self.start_time = self.current_time()     #Helps in movement of micromouse

        self.pos_x = 1       #x coordinate of micromouse
        self.pos_y = 14      #y coordinate of micromouse

        self.heading = 0     #Cardinal direction the micromouse is facing
        self.target_heading = 0     #Direction the micromouse should change to
        """
        0 = N
        1 = E
        2 = S
        3 = W
        """

        self.state = "STOP"
        """
        States: STOP, TURN, FORWARD, SETTLE
        """


        
        self.boundaries()       #Maps outer walls
        self.floodfill()

        self.get_logger().info("Student Solver Node initialized successfully.")
        self.get_logger().info(f"Stats -> Speed: {TOP_SPEED}, Accel: {ACCELARATION}, Turn: {TURN_SPEED}, Range: {SENSOR_RANGE}")



    def boundaries(self):          #Method to map the outer walls of the maze
        for i in range(size):
            wall_vert[i][0] = 1
        for i in range(size):
            wall_vert[i][size] = 1
        for i in range(size):
            wall_horz[0][i] = 1
        for i in range(size):
            wall_horz[size][i] = 1


    #======TIME HELPERS=========
    def current_time(self):             #Returns current timestamp
        return self.get_clock().now().nanoseconds

    def time_elapsed(self):             #Measures time passed since last timestamp
        return (self.current_time() - self.start_time) / 1000000000.0
    #===========================
    

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
        Bonus wall Following algorithm   ;)

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

        if self.state == "FORWARD":

            

            cmd.linear.x = 1.0
            if self.time_elapsed() >= self.cell_time:
                cmd.linear.x = 0.0
                if self.heading == 0:             #Updating location of micromouse
                    self.pos_y -= 1
                elif self.heading == 1:
                    self.pos_x += 1
                elif self.heading == 2:
                    self.pos_y += 1
                elif self.heading == 3:
                    self.pos_x -= 1

                self.state = "STOP"              #Changing state of micromouse
                self.start_time = self.current_time()         #Updating timestamp


        elif self.state == "STOP":
            self.set_wall(self.pos_x, self.pos_y, d_front, d_right, d_left)    #Updating walls of map
            self.floodfill()                   #Updating floodfill map
            self.find_target_heading()         #Finding next direction to move to
            self.state = "TURN"                #Changing state
            self.start_time = self.current_time()       #Updating timestamp

        elif self.state == "TURN":
            self.get_logger().info(
                f"Position = ({self.pos_x}, {self.pos_y}), "
                f"Heading = {self.heading}, "
                f"Sensors = L:{d_left:.2f}, F:{d_front:.2f}, R:{d_right:.2f}"
            )
            self.get_logger().info(
                f"Position = ({self.pos_x}, {self.pos_y}), "
                f"Heading = {self.heading}, "
                f"Target = {self.target_heading}, "
                f"Flood = {flood[self.pos_x][self.pos_y]}"
            )

            if self.heading == self.target_heading:
                self.state = "SETTLE"
                self.start_time = self.current_time()
            else:
                turn = (self.target_heading - self.heading) % 4

                if turn == 1:
                    # Turn right 90 degrees
                    cmd.angular.z = -1.0

                    if self.time_elapsed() >= self.turn_time:
                        self.heading = (self.heading + 1) % 4
                        self.state = "SETTLE"
                        self.start_time = self.current_time()

                elif turn == 3:
                    # Turn left 90 degrees
                    cmd.angular.z = 1.0

                    if self.time_elapsed() >= self.turn_time:
                        self.heading = (self.heading - 1) % 4
                        self.state = "SETTLE"
                        self.start_time = self.current_time()

                elif turn == 2:
                    # Turn 180 degrees
                    cmd.angular.z = -1.0

                    if self.time_elapsed() >= 2 * self.turn_time:
                        self.heading = (self.heading + 2) % 4
                        self.state = "SETTLE"
                        self.start_time = self.current_time()

        elif self.state == "SETTLE":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self.time_elapsed() >= self.settle_time:
                self.state = "FORWARD"
                self.start_time = self.current_time()
                


            

        self.cmd_pub.publish(cmd)
    


    def set_wall(self, x, y, d_front, d_right, d_left):     #Method to update the walls in the maze

        if d_front < self.wall_thresh:        #Updating based on front sensor values
            if self.heading == 0:
                wall_horz[y][x] = 1
            elif self.heading == 2:
                wall_horz[y + 1][x] = 1
            elif self.heading == 1:
                wall_vert[y][x + 1] = 1
            elif self.heading == 3:
                wall_vert[y][x] = 1
        
        if d_right < self.wall_thresh:        #Updating based on right sensor values
            if self.heading == 0:
                wall_vert[y][x + 1] = 1
            elif self.heading == 2:
                wall_vert[y][x] = 1
            elif self.heading == 1:
                wall_horz[y + 1][x] = 1
            elif self.heading == 3:
                wall_horz[y][x] = 1

        if d_left < self.wall_thresh:         #Updating based on left sensor values
            if self.heading == 0:
                wall_vert[y][x] = 1
            elif self.heading == 2:
                wall_vert[y][x + 1] = 1
            elif self.heading == 1:
                wall_horz[y][x] = 1
            elif self.heading == 3:
                wall_horz[y + 1][x] = 1





    def is_wall(self, x, y, d):        #Method to check if there is a wall adjacent to a given cell
        if d == 0:
            if wall_horz[y][x] == 1:
                return True
            else:
                return False
            
        
        elif d == 1:
            if wall_vert[y][x + 1] == 1:
                return True
            else:
                return False

        elif d == 2:
            if wall_horz[y + 1][x] == 1:
                return True
            else:
                return False

        elif d == 3:
            if wall_vert[y][x] == 1:
                return True
            else:
                return False





    def floodfill(self):             #Method to update the floodfill map
        #Clearing the flood array
        for dr in range(size):
            for dc in range(size):
                flood[dr][dc] = -1


        #Setting goal cells to 0
        for dr, dc in self.goals:
            flood[dr][dc] = 0


        #Initializing the queue
        queue = []
        for dx, dy in self.goals:
            queue.append((dx, dy))

        pointer = 0
        while pointer < len(queue):
            x, y = queue[pointer]

            cell_value = flood[x][y]

            for direction in range(4):
                if self.is_wall(x, y, direction):    #Checking if there is a wall adjacent
                    continue

                dr, dc = directions[direction]
                r = x + dr
                c = y + dc
                if 0 <= r < len(flood) and 0 <= c < len(flood[0]) and flood[r][c] == -1:
                    flood[r][c] = cell_value + 1
                    queue.append((r, c))

            pointer += 1        #Dequeue

            




    def find_target_heading(self):
        least_value = 1000000
        self.target_heading = self.heading

        for direction in range(4):
            if self.is_wall(self.pos_x, self.pos_y, direction):
                continue

            dr, dc = directions[direction]
            r = self.pos_x + dr
            c = self.pos_y + dc
            if 0 <= r < len(flood) and 0 <= c < len(flood[0]):
                if flood[r][c] < least_value:
                    least_value = flood[r][c]
                    self.target_heading = direction


        

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