import rospy
import time
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, ColorRGBA, Header
from morai_msgs.msg import CtrlCmd

class PurePursuit:
    def __init__(self, lookahead_distance, max_steering_angle):
        self.lookahead_distance = lookahead_distance
        self.max_steering_angle = max_steering_angle

    def calculate_steering_angle(self, current_pose, path):
        # Initialize closest point and distance to maximum value
        closest_point = None
        closest_distance = float('inf')

        # Find the point on the path closest to the current position
        for point in path:
            # distance = np.linalg.norm(point - current_pose[:2]) # Original code
            norm_input = [point.x - current_pose[0], point.y - current_pose[1]]
            distance = np.linalg.norm(norm_input)
            if distance < closest_distance:
                closest_distance = distance
                closest_point = point

        # Calculate the lookahead point

        closest_current_sub = [closest_point.x - current_pose[0], closest_point.y - current_pose[1]]
        # print("closest")
        # print(closest_current_sub)
        
        # print(closest_point)
        # print(closest_distance)
        
        refined_closest_point = [closest_point.x, closest_point.y]
        closest_distance_list = [closest_distance, closest_distance]
        
        lookahead_distance_list = [self.lookahead_distance, self.lookahead_distance]
        
        distance_mul = [lookahead_distance_list[i] * closest_current_sub[i] for i in range(len(closest_current_sub))]
        
        distance_div = [distance_mul[i] / closest_distance_list[i] for i in range(len(closest_distance_list))]

        # lookahead_point = closest_point + self.lookahead_distance * (closest_point - current_pose[:2]) / closest_distance # Original code
        lookahead_point = [distance_div[i] + refined_closest_point[i] for i in range(len(refined_closest_point))]
        
        #print()
        #print("refined_closest_point")
        #print(refined_closest_point)
        #print("distance_div")
        #print(distance_div)
        #print("self.lookahead_distance")
        #print(self.lookahead_distance)
        #print("closest_current_sub")
        #print(closest_current_sub)
        #print("closest_distance")
        #print(closest_distance)
        #print("current_pose")
        #print(current_pose)
        #print("lookahead_point")
        #print(lookahead_point)

        # Calculate the steering angle
        steering_angle = np.arctan2(lookahead_point[1] - current_pose[1], lookahead_point[0] - current_pose[0]) - current_pose[2]
        #print("steering_angle")
        #print(steering_angle)

        # Limit the steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        return steering_angle

class Controller:
    def __init__(self):
        print("Controller __init__ called.")
        self.path = None
        self.pure_pursuit = PurePursuit(1.0, 0.3)
        self.pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        # self.pub = rospy.Publisher("/ctrl_cmd_0", CtrlCmd, queue_size=1)
        '''
        int32 longlCmdType
        float64 accel
        float64 brake
        float64 steering # positive : left / negative : right
        
        float64 velocity
        float64 acceleration
        '''
        self.path_sub = rospy.Subscriber("/refined_vertices", MarkerArray, self.path_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.debug_marker = rospy.Publisher("/tmp_goal", Marker, queue_size=1)
        
    def remove_closest_vertex(self, pose):
        # Remove Closest Marker in self.path list
        # print("pose")
        # print(pose)
        
        #print("current path len()")
        #print(len(self.path_points))
        
        np_pose = np.array((pose[0], pose[1]))
        for marker in self.path_points:
            vertex_xy = np.array((marker.x, marker.y))
            # print(np.linalg.norm(np_pose - vertex_xy))
            if np.linalg.norm(np_pose - vertex_xy) < 1.5:
                self.path_points.remove(marker)
            #print(marker)
    
    def make_header(self, frame_id, stamp=None):
        if stamp == None:
            stamp = rospy.Time.now()
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header
    
    def path_callback(self, msg):
        print("Path updated.")
        self.path = msg.markers
        self.path_points = [Point(p.points[0].x, p.points[0].y, 0.0) for p in self.path]

        # print(self.path)
        #for marker in self.path:
            #print(marker.points[0].x)
            #print(marker.points[0].y)
        
    def odom_callback(self, msg):
        #start = time.time()
    
        cmd = CtrlCmd()
        if self.path == None or len(self.path_points) == 0:
            pass
        else:
            current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z]
            # path_points = [Point(p.pose.position.x, p.pose.position.y, 0.0) for p in self.path] # Original code
            self.remove_closest_vertex(current_pose)
            # print(current_pose)
            # print(path_points)
            # print("Closest goal")
            # print(self.path_points[0].x, self.path_points[0].y)
            tmp_marker = Marker()
            
            tmp_point = Point()
            tmp_marker.points.append(tmp_point)

            tmp_marker.ns = "tmp_goal"
            tmp_marker.id = 0
            tmp_marker.text = "TMP goal"

            tmp_marker.type = 8
            tmp_marker.lifetime = rospy.Duration.from_sec(3)
            
            tmp_marker.header = self.make_header("map")
            
            tmp_marker.scale.x = 0.8
            tmp_marker.scale.y = 0.8
            tmp_marker.scale.z = 0.1
            
            tmp_marker.color.r = 1.0
            tmp_marker.color.a = 1.0
            
            tmp_marker.points[0].x = self.path_points[0].x
            tmp_marker.points[0].y = self.path_points[0].y
            
            # print(tmp_marker)
            
            self.debug_marker.publish(tmp_marker)
            # print("current pose")
            # print(current_pose)
            steering_angle = self.pure_pursuit.calculate_steering_angle(current_pose, self.path_points)
        
            cmd.accel = 0.3
            cmd.steering = steering_angle
            
            # In Simulation, cmd.steering range -1.0 ~ 1.0
            # If steering in Python code is 0.1,
            # in simulation, converts -0.2(left)
            
            # cmd.steering = 0.1
            # print(-steering_angle)
            print("steering_angle")
            print(steering_angle)
        
            self.pub.publish(cmd)
            #end = time.time()
            #print(end - start)
        
if __name__=="__main__":
        rospy.init_node("Pure_pursuit")
        try:
            Controller()
            rospy.spin()
        except:
            pass
