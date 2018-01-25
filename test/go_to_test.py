#! /usr/bin/python

import sys
import rospy
from geometry_msgs.msg import Pose
from denso_robot_interface.srv import *

def pose_test():
  rospy.wait_for_service('/denso_robot_interface/go_to')
  try:
    go_to = rospy.ServiceProxy('/denso_robot_interface/go_to', GoTo)
    request = GoToRequest()
    request.next = Pose()
    request.next.position.x = 0.296421666232
    request.next.position.y = -0.234169975638
    request.next.position.z = 0.365434782962
    request.next.orientation.x = 0.000202591058181
    request.next.orientation.y = -0.999961585071
    request.next.orientation.z = -0.00525003584879
    request.next.orientation.w = 0.00701601500909
    respond = go_to(request)
    return respond
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e

def translation_test():
  rospy.wait_for_service('/denso_robot_interface/translation')
  try:
    translate = rospy.ServiceProxy('/denso_robot_interface/translation', Translation)
    request = TranslationRequest()
    request.trajectory = [0.35, -0.234169975638, 0.365434782962, 0.35, -0.15, 0.365434782962]
    respond = translate(request)
    return respond
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e



if __name__ == "__main__":
  print ("Testing pose service...")
  respond = pose_test()
  print (respond.reason)

  print ("Testing translation service...")
  respond = translation_test()
  print (respond.reason)
