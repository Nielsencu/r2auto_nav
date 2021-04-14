# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from .a_star import a_star_search
from math import atan2
import scipy.stats
from .rviz import RvizInterface
from PIL import Image 
import matplotlib.pyplot as plt
from std_msgs.msg import String


# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 50, 100]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
map_bg_color = 1
count = 0

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning

        self.string_subscription = self.create_subscription(
            String,
            'shooter',
            self.string_callback,
            10)

        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.laser_range = np.array([])

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        self.visited = []
        self.frontierpoints = []

        self.x = None
        self.y = None

        self.path = []
        self.goal = Point()
        self.nearest_frontier = Point()
        self.shooterFlag = False
        self.shooterDirection = 'forward'
        self.angle_to_goal = -1
 
    def string_callback(self,msg):
        self.get_logger().info('In shooter callback %s' % msg.data)
        if msg.data == 'Stop moving':
            self.shooterFlag = True
            self.get_logger().info('Stopping because target detected')
        elif msg.data == 'Continue moving':
            self.shooterFlag = False
            self.get_logger().info('Resuming auto_nav ...')
        elif msg.data == 'left':
            self.shooterDirection = 'left'
            self.get_logger().info('shuter ask me go left')
        elif msg.data == 'right':
            self.shooterDirection = 'right'
            self.get_logger().info('shoter ask me go right')
        elif msg.data == 'forward':
            self.shooterDirection = 'forward'
            self.get_logger().info('shuter ask me go forward')
        elif msg.data == 'stop':
            self.shooterDirection = 'stop'
            self.get_logger().info('shoter ask me stop')
    
    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
    
    def odom_callback(self, msg):
        #self.get_logger().info('In odom_callback')

        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)      
    
    def find_unoccupied(self,pos):
        def is_frontier(i,j):
            neighbors = ((i,j) , (i,j+1), (i + 1,j + 1), (i + 1,j), (i+1,j -1 ), (i,j -1) , (i-1,j-1), (i-1,j), (i-1,j + 1))
            for (a,b) in neighbors:
                if self.occdata[b][a] in (2,3):
                    return False
            return True
        queue = []
        queue.append((pos[0],pos[1]))
        visited = {}
        #condition = True
        while len(queue) > 0:
            current_pos = queue.pop(0)
            x = current_pos[0]
            y = current_pos[1]
            visited[current_pos] = 1
            if self.occdata[y][x] == 3: # Flag this as an obstacle, no need to check if its an frontier
                continue
            elif is_frontier(x,y):
                return (x,y)
            neighbors = ((x,y+1) , (x,y-1) , (x+1,y) , (x-1,y))
            for neighbor in neighbors:
                if neighbor not in queue and visited.get(neighbor) == None:
                    queue.append(neighbor)
        return None

    def occ_callback(self, msg):
        self.get_logger().info('In occ_callback')
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        iwidth = msg.info.width
        iheight = msg.info.height
        
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        # calculate total number of bins
        total_bins = msg.info.width * msg.info.height
        
        # make msgdata go from 0 instead of -1, reshape into 2D
        
        # reshape to 2D array using column order
        self.occdata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))

        # #Inflating obstacles
        # obstacles = []
        # for i in range(len(self.occdata) -1):
        #     for j in range(len(self.occdata[0])-1):
        #         if self.occdata[i][j] == 3:
        #             obstacles.append((i,j))
        # for i in obstacles:
        #     self.occdata[i[0] + 1][i[1]] = 3
        #     self.occdata[i[0] - 1][i[1]] = 3
        #     self.occdata[i[0]][i[1] + 1] = 3
        #     self.occdata[i[0]][i[1] - 1] = 3
                    
        #get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout = rclpy.time.Duration())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(e)
            self.get_logger().info('No transformation found')
            self.stopbot()
            return
                
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation

        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))

        self.x = grid_x
        self.y = grid_y

        # set current robot location to 0
        self.occdata[grid_y][grid_x] = 0
        # Pop current goal because it has been reached
        if self.shooterFlag == True:
            return


        if distance((self.x,self.y), (self.goal.x , self.goal.y)) <= 1.5:

            self.stopbot()
            
            twist = Twist()
            self.get_logger().info('Start moving after popping')
            twist.linear.x = 0.05
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            
            
            rclpy.spin_once(self)
            
            print("Path popped : " ,self.path.pop(0))

            # If there's no path, but nearest_frontier is not None
            if not(self.path) :
                if not(self.nearest_frontier.x == -1.0 and self.nearest_frontier.y == -1.0):
                    self.path = a_star_search(self.occdata,(self.x , self.y), (int(self.nearest_frontier.x) , int(self.nearest_frontier.y)))

            print("path now is : ", self.path)

        if not(self.path):
            self.get_logger().info('Empty path')
            return
        
        # Setting new goal
        self.goal.x = float(self.path[0][1])
        self.goal.y = float(self.path[0][0])

        inc_x = self.goal.x - self.x
        inc_y = self.goal.y - self.y
        # Get angle to rotate to current goal
        self.angle_to_goal = (atan2(inc_y,inc_x))

        self.get_logger().info("Map size is %i %i " % (len(self.occdata), len(self.occdata[0])))
        self.get_logger().info("Position now is %i %i " % (grid_x,grid_y))

        if(self.occdata.any()):
            visualization = RvizInterface(self.occdata, msg)

        if self.path:
            self.get_logger().info("Hey path is published")
            visualization.publishPath(self.path)

        # create image from 2D array using PIL
        img = Image.fromarray(self.occdata)
        # find center of image
        i_centerx = iwidth/2
        i_centery = iheight/2
        # find how much to shift the image to move grid_x and grid_y to center of image
        shift_x = round(grid_x - i_centerx)
        shift_y = round(grid_y - i_centery)
        # self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))

        # pad image to move robot position to the center
        # adapted from https://note.nkmk.me/en/python-pillow-add-margin-expand-canvas/ 
        left = 0
        right = 0
        top = 0
        bottom = 0
        if shift_x > 0:
            # pad right margin
            right = 2 * shift_x
        else:
            # pad left margin
            left = 2 * (-shift_x)
            
        if shift_y > 0:
            # pad bottom margin
            bottom = 2 * shift_y
        else:
            # pad top margin
            top = 2 * (-shift_y)
            
        # create new image
        new_width = iwidth + right + left
        new_height = iheight + top + bottom
        img_transformed = Image.new(img.mode , (new_width, new_height), 1)
        img_transformed.paste(img, (left, top))
        img_transformed = img_transformed.resize((img_transformed.size[0] * 20 , img_transformed.size[1] * 20))
        

        # rotate by 90 degrees so that the forward direction is at the top of the image
        #rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)

        # show the image using grayscale map
        # plt.imshow(img, cmap='gray', origin='lower')
        # plt.imshow(img_transformed, cmap='gray', origin='lower')
        plt.imshow(img_transformed, origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)

        # print to file
        np.savetxt(mapfile, self.occdata, fmt='%d', delimiter='')


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        
        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

        # function to rotate the TurtleBot
    def rotate360(self):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        twist.linear.x = 0.0
        twist.angular.z = 0.3
        time.sleep(1)
        self.publisher_.publish(twist)
        print("Current yaw is ", current_yaw)
        time.sleep(2)
        rclpy.spin_once(self)

        while abs(self.yaw - current_yaw) > 0.1:
            if self.shooterFlag == True:
                print(" i exit because detected")
                break
            twist = Twist()
            rclpy.spin_once(self)
            print("Still rotating 360" , self.yaw , current_yaw)
            twist.linear.x = 0.0
            twist.angular.z = 0.3
            self.publisher_.publish(twist)
            
        print("Exited 360 loop")

    def pick_direction(self):
        print("Goal now is", self.goal.x , self.goal.y)
        print("Now i'm at ", self.x , self.y)
        print("I want go to :", self.angle_to_goal, "I'm facing", self.yaw)

        twist = Twist()

        while abs(self.yaw - self.angle_to_goal) > 0.2:
            print(" changing direction" , self.yaw , self.angle_to_goal)
            difference = self.yaw - self.angle_to_goal
            print(" difference" , difference)
            if difference < 0:
                if(difference < -math.pi):
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3

            elif difference > 0:
                if(difference > math.pi):
                    # Fast turn
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                else:
                    # Gentle turn
                    twist.linear.x = 0.0
                    twist.angular.z = -0.3
            
            self.publisher_.publish(twist)
            rclpy.spin_once(self)

        rclpy.spin_once(self)
    
        # start moving
        self.get_logger().info('Start moving')
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        self.publisher_.publish(twist) 

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def getPath(self):
        self.path = a_star_search(self.occdata,(self.x , self.y), (int(self.nearest_frontier.x) , int(self.nearest_frontier.y)))
        if(not(self.path)):
            # stop, wait for transformation from tf tree
            twist = Twist()
            self.get_logger().info('GOD DAYM CANT FIND DESTINATION')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

        # Rotating 360
        self.rotate360()

        rclpy.spin_once(self) 

    def mover(self):
        global count
        try:
            rclpy.spin_once(self)
            current = (self.x,self.y)

            while rclpy.ok():
                # allow the callback functions to run
                rclpy.spin_once(self)

                if self.shooterFlag == True:
                    twist = Twist()
                    if self.shooterDirection == 'forward':
                        self.get_logger().info(" forward in mover")
                        twist.linear.x = -0.1
                        twist.angular.z = 0.0
                    elif self.shooterDirection == 'left':
                        self.get_logger().info("left in mover")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.1
                    elif self.shooterDirection == 'right':   
                        self.get_logger().info(" right in mover")
                        twist.linear.x = 0.0
                        twist.angular.z = -0.1
                    elif self.shooterDirection == 'stop':
                        self.get_logger().info(" stopping in mover")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    self.publisher_.publish(twist)                     
                    rclpy.spin_once(self)
                    continue

                current = (self.x,self.y)
                if self.x == None or self.y == None:
                    print("Robot's coordinates not detected")
                else:
                    print("Finding nearest frontier")

                    nearest_frontier = self.find_unoccupied(pos=current)

                    if not(nearest_frontier): 
                        self.nearest_frontier.x = -1.0
                        self.nearest_frontier.y = -1.0

                        if not(self.path): 
                            # If cannot find nearest frontier 10 consecutive tries, and don't have old self.path, maps completed
                            count+=1
                            if count == 10:
                                self.get_logger().info('Map is completed')
                                return
                            rclpy.spin_once(self)
                            continue
                        
                    else:
                        self.nearest_frontier.x = float(nearest_frontier[0])
                        self.nearest_frontier.y = float(nearest_frontier[1])
                        count = 0

                    print("Hey my position in mover is,",current)                        
                    print("Nearest frontier is " ,self.nearest_frontier.y , self.nearest_frontier.x)
                    print("My goal is ", self.goal.y , self.goal.x)
                    print("Path is :" ,self.path)

                    if self.path:
                        self.goal.x = float(self.path[0][1])
                        self.goal.y = float(self.path[0][0])
                        acc = 0
                        for (y,x) in self.path:
                            # If path is no longer valid, update it
                            if(self.occdata[y][x] == 3): 
                                self.rotate360()
                                self.get_logger().info("Updating path, because it is blocked ...")
                                self.getPath()
                                break
                            elif self.occdata[y][x] == 1:
                                acc +=1
                        # If path doesnt have -1 anymore, update it to find new frontier
                        if acc == 0:
                            self.rotate360()
                            self.get_logger().info("Updating path, because no longer unmapped")
                            self.getPath()
                            continue
                        
                        # If current position is nearer to final goal than 0th index in path to final goal, update path
                        if distance((self.x , self.y) , (self.path[-1])) - distance(self.path[0] , self.path[-1]) < -1:
                            # Rotate 360
                            self.rotate360()
                            self.get_logger().info("Updating path, because found a closer path")
                            self.getPath()

                        self.pick_direction()

                    else:
                        # Rotating 360 everytime finds a new path
                        self.rotate360()

                        # Find path if self.path is not there
                        self.getPath()


        except Exception as e:
            print(e)
            print("Exception")
        # # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()
                

def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

def distance(point1,point2):
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 )**0.5
