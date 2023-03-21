#!/usr/bin/env python


import collections
import sys

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

from visualization_msgs.msg import Marker, MarkerArray

import utils
import math

# The topic to publish control commands to
PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'
initialpose_topic = '/initialpose'
'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
def L2dist(a, b): #return L2/eucleadian distance between a and b
  pa = np.array(a)
  pb = np.array(b)
  return np.linalg.norm(pa-pb)

class LineFollower:
  '''
  Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation
    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should trave
  '''
  def __init__(self, plan, pose_topic, plan_lookahead, translation_weight,
               rotation_weight, kp, ki, kd, error_buff_length, speed):
    # Store the passed parameters
    self.plan = plan
    self.plan_lookahead = plan_lookahead
    # Normalize translation and rotation weights
    self.translation_weight = translation_weight / (translation_weight+rotation_weight)
    self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    # The error buff stores the error_buff_length most recent errors and the
    # times at which they were received. That is, each element is of the form
    # [time_stamp (seconds), error]. For more info about the data struct itself, visit
    # https://docs.python.org/2/library/collections.html#collections.deque
    self.error_buff = collections.deque(maxlen=error_buff_length)
    self.speed = speed
    
    # YOUR CODE HERE
    self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=10) # Create a publisher to PUB_TOPIC
    self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb, queue_size=10) # Create a subscriber to pose_topic, with callback 'self.pose_cb'
  
  

    # min distances measurement from true car pose to every waypoint
    self.good_points = None # this will be a np.array
    self.bad_points = None #this will be a np.array
    self.good_distances = None
    self.bad_distances = None
    good_waypoint_topic = "/waypoint/good"
    bad_waypoint_topic = "/waypoint/bad"
    self.carpose2waypoints_sub = rospy.Subscriber('/car/car_pose', PoseStamped, self.dist2waypoints, queue_size=10) 
    self.dist2good_sub = rospy.Subscriber(good_waypoint_topic, MarkerArray, self.dist2good_cb, queue_size=10)  
    self.dist2bad_sub = rospy.Subscriber(bad_waypoint_topic, MarkerArray, self.dist2bad_cb, queue_size=10) 
  '''
  plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
  '''

  def dist2good_cb(self, msg):
    if self.good_points is None:
      temp = []
      for marker in msg.markers:
        xy_pose = [marker.pose.position.x, marker.pose.position.y]
        temp.append(xy_pose)
      self.good_points = np.array(temp)
      self.good_distances = np.full(len(self.good_points), 99999.9)

  def dist2bad_cb(self, msg):
    if self.bad_points is None:
      temp = []
      for marker in msg.markers:
        xy_pose = [marker.pose.position.x, marker.pose.position.y]
        temp.append(xy_pose)
      self.bad_points = np.array(temp)
      self.bad_distances = np.full(len(self.bad_points), 99999.9)

  def dist2waypoints(self, msg): #measure car pose to every
    if self.good_distances is not None and self.bad_distances is not None:
      cur_carpose = np.array([msg.pose.position.x, msg.pose.position.y])
      for i in range(len(self.good_distances)):
        d = L2dist(cur_carpose, self.good_points[i])
        if d < self.good_distances[i]:
          self.good_distances[i] = d

      for i in range(len(self.bad_distances)):
        d = L2dist(cur_carpose, self.bad_points[i])
        if d < self.bad_distances[i]:
          self.bad_distances[i] = d

  def report_mindist_car2waypoints(self):
    for i in range(len(self.good_distances)):
      print('the min distance from true car_pose to goodpoint ', i, ' is: ', self.good_distances[i])
    for i in range(len(self.bad_distances)):
      print('the min distance from true car_pose to badpoint ', i, ' is: ', self.bad_distances[i])
      
  def compute_error(self, cur_pose):
    
    # Find the first element of the plan that is in front of the robot, and remove
    # any elements that are behind the robot. To do this:
    # Loop over the plan (starting at the beginning) For each configuration in the plan
        # If the configuration is behind the robot, remove it from the plan
        #   Will want to perform a coordinate transformation to determine if 
        #   the configuration is in front or behind the robot
        # If the configuration is in front of the robot, break out of the loop
    #print(self.plan)
    x0 = cur_pose[0]
    y0 = cur_pose[1]
    theta0 = cur_pose[2]

    while len(self.plan) > 0:
        
        #rot = np.array(np.matmul(utils.rotation_matrix(cur_pose[2]), [[self.plan[0][0] - cur_pose[0]],[self.plan[0][1] - cur_pose[1]]]))
        car_cooridinate = np.array([np.cos(theta0), np.sin(theta0)])
        plan_car_vec = np.array([self.plan[0][0] - x0, self.plan[0][1] - y0])
        rot = np.dot(car_cooridinate, plan_car_vec)
        
        if (rot < 0):
            self.plan.pop(0)
        else: 
            break
      
    # Check if the plan is empty. If so, return (False, 0.0)
    # YOUR CODE HERE
    if (len(self.plan) <= 0): 
      return (False, 0.0)
    
    # At this point, we have removed configurations from the plan that are behind
    # the robot. Therefore, element 0 is the first configuration in the plan that is in 
    # front of the robot. To allow the robot to have some amount of 'look ahead',
    # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
    # We call this index the goal_index
    goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)
    x_goal = self.plan[goal_idx][0]
    y_goal = self.plan[goal_idx][1]
    theta_goal = self.plan[goal_idx][2]
    # Compute the translation error between the robot and the configuration at goal_idx in the plan
    # YOUR CODE HERE
    #translation_error = -np.array(np.matmul(
    # utils.rotation_matrix(cur_pose[2]), np.array([[self.plan[int(goal_idx)][0] - cur_pose[0]], [self.plan[int(goal_idx)][1] - cur_pose[1]]])))[0]
    goal_vec = np.array([x_goal-x0, y_goal-y0])
    car_left_vec = np.array(np.matmul(utils.rotation_matrix(-np.pi/2), car_cooridinate.reshape(2,1)))
    translation_error = np.dot(goal_vec, car_left_vec)
    
    # Compute the total error
    # Translation error was computed above
    # Rotation error is the difference in yaw between the robot and goal configuration
    #   Be carefult about the sign of the rotation error
    # YOUR CODE HERE
    # 
    
    rotation_error = theta_goal - theta0 
    if rotation_error > np.pi:
      rotation_error = rotation_error - 2*np.pi
    elif rotation_error < -np.pi:
      rotation_error = rotation_error + 2*np.pi
    else:
      pass
  
    error = self.translation_weight * translation_error + self.rotation_weight * rotation_error
    
    
    return (True, error)
    
    
  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''
  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec() # Get the current time
    
    # Compute the derivative error using the passed error, the current time,
    # the most recent error stored in self.error_buff, and the most recent time
    # stored in self.error_buff
    # YOUR CODE HERE
    # derivaticve error = kd * de(t) / dt , pass error, current time
    deriv_error = 0
    integ_error = 0
    # Add the current error to the buffer
    self.error_buff.append((error, now))
    #print('error', self.error_buff[-1])


    if self.error_buff[-1][1] == now:
      pass
    else:
      deriv_error = (self.error_buff[-1][0]-error)/(self.error_buff[-1][1]-now)
    # Compute the integral error by applying rectangular integration to the elements
    # of self.error_buff: https://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/
    # YOUR CODE HERE
  
    for num in self.error_buff[0]:
      integ_error += num
    integ_error *= (self.error_buff[-1][1]-now)
    # Compute the steering angle as the sum of the pid errors
    # YOUR CODE HERE
    pid_error = - self.kp*error + self.ki*integ_error - self.kd * deriv_error
    print(float(pid_error))
    print(',')
    return pid_error
    
  '''
  Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, but feel free to change it
  '''  
  def pose_cb(self, msg):
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])

    success, error = self.compute_error(cur_pose)
    
    if not success:
      # We have reached our goal
      self.pose_sub = None # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops
      
    delta = self.compute_steering_angle(error)
    
    # Setup the control message
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed
    
    # Send the control message
    self.cmd_pub.publish(ads)
    #print(ads)

def main():

  rospy.init_node('line_follower', anonymous=True) # Initialize the node
  
  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LineFollower class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system
  # YOUR CODE HERE
  plan_topic = rospy.get_param("~plan_topic") # Default val: '/planner_node/car_plan'
  pose_topic = rospy.get_param("~pose_topic") # Default val: '/sim_car_pose/pose'
  plan_lookahead = rospy.get_param("~plan_lookahead")# Starting val: 5
  translation_weight = rospy.get_param("~translation_weight") # Starting val: 1.0
  rotation_weight = rospy.get_param("~rotation_weight")# Starting val: 0.0
  kp = rospy.get_param("~kp")# Startinig val: 1.0
  ki = rospy.get_param("~ki")# Starting val: 0.0
  kd = rospy.get_param("~kd")# Starting val: 0.0
  error_buff_length = rospy.get_param("~error_buff_length")# Starting val: 10
  speed = rospy.get_param("~speed") # Default val: 1.0

  raw_input("Press Enter to when plan available...")  # Waits for ENTER key press

  initialpose_pub = rospy.Publisher(initialpose_topic, PoseWithCovarianceStamped, queue_size=10)
  
  pcs = PoseWithCovarianceStamped()
  pcs.header.frame_id = "map"
  pcs.header.stamp = rospy.Time()
  pcs.pose.pose.position.x = 49.9
  pcs.pose.pose.position.y = 11.938
  pcs.pose.pose.position.z = 0.0
  pcs.pose.pose.orientation.x = 0.0
  pcs.pose.pose.orientation.y = 0.0
  pcs.pose.pose.orientation.z = -0.105
  pcs.pose.pose.orientation.w = 0.995
  rospy.sleep(1.0) 
  initialpose_pub.publish(pcs)
  # Use rospy.wait_for_message to get the plan msg
  # Convert the plan msg to a list of 3-element numpy arrays
  #     Each array is of the form [x,y,theta]
  # Create a LineFollower object
  # YOUR CODE HERE
  print('Waiting for the plan to be published from PlannerNode, plan_topic: ', plan_topic)
  converted_plan = []
  for msg in rospy.wait_for_message('/planner_node/car_plan', PoseArray).poses:
    converted_plan.append([msg.position.x, msg.position.y, utils.quaternion_to_angle(msg.orientation)])

  print('Plan is generated, car should be starting to drive along with planned path')
  
  LineFollower(converted_plan, pose_topic, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd, error_buff_length, speed)

  '''
  plt.plot(lf.xpoints, lf.ypoints,'bo',ls='-')
  plt.xlabel('iterration index')
  plt.ylabel('PID error')
  plt.show()
  '''
  rospy.spin() # Prevents node from shutting down

if __name__ == '__main__':
  main()
