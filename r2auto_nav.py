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
from geometry_msgs.msg import Twist
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

# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 50, 100]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

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
        self.laser_range = np.array([])

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        self.visited = []
        self.frontierpoints = []

        self.x = None
        self.y = None

        self.path = []
        self.goal = None
 
        
    def odom_callback(self, msg):
        #self.get_logger().info('In odom_callback')

        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def get_path(self,current,destination):
        pass        
    
    def find_unoccupied(self,pos):
        def is_frontier(x,y):
            if self.occdata[y][x] in (2,3):
                return False
            neighbors = ((x,y+1) , (x,y-1) , (x+1,y) , (x-1,y))
            for i in neighbors:
                #print(i, self.occdata[i[1]][i[0]])
                if self.occdata[i[1]][i[0]] in (2,3):
                    return False
            return True
        queue = []
        queue.append((pos[0],pos[1]))
        visited = []
        #condition = True
        while len(queue) > 0:
            current_pos = queue.pop(0)
            x = current_pos[0]
            y = current_pos[1]
            visited.append(current_pos)
            if self.occdata[y][x] == 3: # Flag this as an obstacle, no need to check if its an frontier
                continue
            elif is_frontier(x,y):
                return (x,y)
            neighbors = ((x,y+1) , (x,y-1) , (x+1,y) , (x-1,y))
            for neighbor in neighbors:
                if neighbor not in queue and neighbor not in visited:
                    queue.append(neighbor)
        return None

    def occ_callback(self, msg):
        self.get_logger().info('In occ_callback')
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        # calculate total number of bins
        total_bins = msg.info.width * msg.info.height
        
        # make msgdata go from 0 instead of -1, reshape into 2D
        
        # reshape to 2D array using column order
        self.occdata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))

        #Inflating obstacles
        obstacles = []
        for i in range(len(self.occdata) -1):
            for j in range(len(self.occdata[0])-1):
                if self.occdata[i][j] == 3:
                    obstacles.append((i,j))
        for i in obstacles:
            self.occdata[i[0] + 1][i[1]] = 3
            self.occdata[i[0] - 1][i[1]] = 3
            self.occdata[i[0]][i[1] + 1] = 3
            self.occdata[i[0]][i[1] - 1] = 3
                    
        #get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
                
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))

        self.x = grid_x
        self.y = grid_y

        # set current robot location to 0
        self.occdata[grid_y][grid_x] = 0

        self.get_logger().info("Map size is %i %i " % (len(self.occdata), len(self.occdata[0])))
        self.get_logger().info("Position now is %i %i " % (grid_x,grid_y))

        visualization = RvizInterface(self.occdata, msg)

        if self.path:
            self.get_logger().info("Hey path is published")
            visualization.publishPath(self.path)

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

    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1
            current = (self.x,self.y)
            # self.pick_direction()

            while rclpy.ok():
                current = (self.x,self.y)
                print("Hey my position in mover is,",current)
                if self.x == None and self.y == None:
                    print("Robot's coordinates not detected")
                else:
                    nearest_frontier = self.find_unoccupied(pos=current)
                    if nearest_frontier == None:
                        self.get_logger().info('Map is completed')
                        return
                    print("Nearest frontier is " ,nearest_frontier)
                    
                    print("Path is :" ,self.path)
                    if self.path:
                        goal = self.path[0]
                        inc_x = goal[0] - self.x
                        inc_y = goal[1] - self.y
                        angle_to_goal = atan2(inc_y,inc_x)
                        angle_in_deg = angle_to_goal * 180 / 3.142
                        
                        print(angle_to_goal, self.yaw)
                        while abs(self.yaw - angle_to_goal) > 0.1:
                            self.rotatebot(10)
                            rclpy.spin_once(self)   

                        print("Goal now is", goal[0] , goal[1])
                        print("Now i'm at ", self.x , self.y)
                        if self.x == goal[0] and self.y == goal[1]:
                            print("Goal reached")
                            self.stopbot()
                            self.path.pop(0)
                        else:
                            # start moving
                            self.get_logger().info('Start moving')
                            twist = Twist()
                            twist.linear.x = speedchange
                            twist.angular.z = 0.0
                            time.sleep(1)
                            self.publisher_.publish(twist)                        

                        # not sure if this is really necessary, but things seem to work more
                        # reliably with this
                    else:
                        self.path = a_star_search(self.occdata, current, nearest_frontier)
                rclpy.spin_once(self)   
               # allow the callback functions to run
        except Exception as e:
            print(e)
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
