#! /usr/bin/python
# This is the script for ps2-2
# In this part, you will need to send pose command to the
# robot controller and move the manipulator according to
# the trajectory in ps1

# if you are using python3, then you don't need this line
from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import Pose
from denso_robot_interface.srv import *

# create a class for this node
class MoveItClient():
  def __init__(self):
    # a service client that connects to the 'go_to' and 'translate' service
    self.go_to = rospy.ServiceProxy('/denso_robot_interface/go_to', GoTo)
    self.translate = rospy.ServiceProxy('/denso_robot_interface/translation', Translation)

    # approach pose (all pose read from your txt file should be relative to this pose)
    approach_position = rospy.get_param('/ps2_2/approach_pose/position')
    approach_orientation = rospy.get_param('/ps2_2/approach_pose/orientation')
    self.approach_pose = Pose()
    self.approach_pose.position.x = approach_position[0]
    self.approach_pose.position.y = approach_position[1]
    self.approach_pose.position.z = approach_position[2]
    self.approach_pose.orientation.x = approach_orientation[0]
    self.approach_pose.orientation.y = approach_orientation[1]
    self.approach_pose.orientation.z = approach_orientation[2]
    self.approach_pose.orientation.w = approach_orientation[3]

  # this is where we do everything
  def move_it(self):
    ####### INSERT YOUR CODE HERE ########
    # we will move to the approach_pose first
    trajectory = [self.approach_pose]
    # this is where you read the txt file

    # this is where you transfer the X, Y, Z
    # value into a geometry_msgs.msg.Pose object
    # (DON'T FORGET TO TAKE APPROACH_POSE INTO ACCOUNT
    #  WHEN CALCULATING X, Y, Z)

    # fill the trajectory with a list of Pose object
    # you wish to execute

    ######################################

    # sending the trajectory to the MoveIt interface
    try:
      trj_index = 1
      request = GoToRequest()
      for waypoint in trajectory:
        # make sure we are dealing with a Pose object
        assert isinstance(waypoint, geometry_msgs.msg._Pose.Pose)
        # fill in the request
        request.next = waypoint
        # send the request
        print ('>>> Executing waypoint %d...'.format(trj_index), end = '')
        respond = self.go_to(request)
        # chech whether the execution is a success
        if respond.success == True:
          print ('Done!')
          continue
        else:
          print ('ERROR!')
          print ('<<< %s'.format(respond.reason))
          raise AttributeError
    except (AssertionError):
      print ('[Exception] Trajectory should be Pose object.')
    except (AttributeError):
      print ('[Exception] ERROR occurred during execution.')
    else:
      print ('>>> All waypoints executed!!')


if __name__ == "__main__":
  rospy.init_node('ps2_2')

  controller = MoveItClient()
  controller.move_it()

  sys.exit(0)
