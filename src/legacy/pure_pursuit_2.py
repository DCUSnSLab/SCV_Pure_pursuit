import rospy
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from morai_msgs.msg import CtrlCmd

class PurePursuit:
    def __init__(self, lookahead_distance, max_steering_angle):
        self.lookahead_distance = lookahead_distance
        self.max_steering_angle = max_steering_angle
        self.path_sub = rospy.Subscriber('/refined_vertices', MarkerArray, self.path_callback)
        self.steering_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)

    def path_callback(self, msg):
        path = []
        for marker in msg.markers:
            path.append(np.array([marker.pose.position.x, marker.pose.position.y]))
        self.path = np.array(path)

    def calculate_steering_angle(self, current_pose):
        # Initialize closest point and distance to maximum value
        closest_point = None
        closest_distance = float('inf')

        # Find the point on the path closest to the current position
        for point in self.path:
            distance = np.linalg.norm(point - current_pose[:2])
            if distance < closest_distance:
                closest_distance = distance
                closest_point = point

        # Calculate the lookahead point
        lookahead_point = closest_point + self.lookahead_distance * (closest_point - current_pose[:2]) / closest_distance

        # Calculate the steering angle
        steering_angle = np.arctan2(lookahead_point[1] - current_pose[1], lookahead_point[0] - current_pose[0]) - current_pose[2]

        # Limit the steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # Publish the steering angle
        steering_msg = Point()
        cmd = CtrlCmd()
        
        cmd.accel = 0.35
        cmd.steering = steering_angle
        
        steering_msg.x = steering_angle
        self.steering_pub.publish(cmd)

        return steering_angle

# Example usage
rospy.init_node('pure_pursuit')
pure_pursuit = PurePursuit(lookahead_distance=1.0, max_steering_angle=np.pi/4)
current_pose = np.array([0, 0, 0])

# Create a timer to update the steering angle at a fixed rate
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pure_pursuit.calculate_steering_angle(current_pose)
    rate.sleep()
