#title           :CloPeMaManipulator.py
#description     :This class is an implementation of "Robot specific layer". The class implements Robot interface
#                 for robotic device used in CTU in project CloPeMa
#author          :Jan Sindler
#conact          :sidnljan@fel.cvut.cz
#date            :20130521
#version         :1.0
#usage           :cannot be used alone
#notes           :
#python_version  :2.7.3  
#==============================================================================

import roslib; roslib.load_manifest('contour_model_folding')
from RobInt import RobInt
import rospy
import cv
from cv_bridge import CvBridge, CvBridgeError
import logging
import sys
from visual_feedback_utils import Vector2D
import numpy as np
from sensor_msgs.msg import Image

# petrik script imports
import rospy, smach, smach_ros, math, copy, tf, PyKDL, os, shutil, numpy
from tf.transformations import quaternion_from_euler, quaternion_about_axis
from tf_conversions import posemath
from clopema_smach import *
from geometry_msgs.msg import *
from smach import State

# includes for pointcloud manipulation
from sensor_msgs.msg        import PointCloud2, PointField
from python_msg_conversions import pointclouds


# HumanManipulator class represent robotic manipualtor simulated by human being.
      
class CloPeMaManipulator(RobInt):  

    lastImageIndex = 0
    graspPoints = None
    pcMap = None # point cloud map. Contain registred point cloud for measuring
    
    x_border = 30
    y_border = 30
      
    def liftUp(self, liftPoints):
        self.graspPoints = liftPoints
        #raw_input("Hit key to continue (liftUp)")
                    
    def place(self, targPoints):
        midP = (map(lambda a:a * 0.5, Vector2D.pt_diff(self.graspPoints[0], targPoints[0])))
        firstLnPt = [self.graspPoints[0][0] + midP[0], self.graspPoints[0][1] + midP[1]]
        midP = (map(lambda a:a * 0.5, Vector2D.pt_diff(self.graspPoints[1], targPoints[1])))
        secondLnPt = [self.graspPoints[1][0] + midP[0], self.graspPoints[1][1] + midP[1]]
        realflp = self.pcMap[firstLnPt[0] + self.x_border, firstLnPt[1] + self.y_border]  #first line point in xtion1_link base
        realslp = self.pcMap[secondLnPt[0] + self.x_border, secondLnPt[1] + self.y_border] #second line point in xtion1_link base    
        
        realTarP1 = self.pcMap[targPoints[0][0] + self.x_border, targPoints[0][1] + self.y_border]
        realTarP2 = self.pcMap[targPoints[1][0] + self.x_border, targPoints[1][1] + self.y_border]
        
        # check none values and reaplce them by nearest neighbour
        search_box_x_half_size = 100;
        search_box_y_half_size = 100;
        realTarP1 = self.checkAndReplaceNaN(realTarP1, targPoints[0][0] + self.x_border, search_box_x_half_size, targPoints[0][1] + self.y_border, search_box_y_half_size)
        realTarP2 = self.checkAndReplaceNaN(realTarP2, targPoints[1][0] + self.x_border, search_box_x_half_size, targPoints[1][1] + self.y_border, search_box_y_half_size)
        realflp = self.checkAndReplaceNaN(realflp, firstLnPt[0] + self.x_border, search_box_x_half_size, firstLnPt[0] + self.y_border, search_box_y_half_size)
        realslp = self.checkAndReplaceNaN(realslp, secondLnPt[0] + self.x_border, search_box_x_half_size, secondLnPt[1] + self.y_border, search_box_y_half_size)
        
        sm = smach.Sequence(outcomes=['succeeded', 'preempted', 'aborted'], connector_outcome='succeeded')
        input_frame = '/xtion1_rgb_optical_frame'
        input_g1 = PyKDL.Frame()
        input_g2 = PyKDL.Frame()
        input_p1 = PyKDL.Frame()
        input_p2 = PyKDL.Frame()

        print realTarP1
        print realTarP2
        print realflp
        print realslp
        input_g1.p = PyKDL.Vector(realTarP1[0], realTarP1[1], realTarP1[2])
        input_g1.M = PyKDL.Rotation().RPY(math.pi / 2, 0, math.pi / 5)
        input_g2.p = PyKDL.Vector(realTarP2[0], realTarP2[1], realTarP2[2])
        input_g2.M = PyKDL.Rotation().RPY(math.pi / 2, 0, -math.pi / 5)
        input_p1.p = PyKDL.Vector(realflp[0], realflp[1], realflp[2])
        input_p2.p = PyKDL.Vector(realslp[0], realslp[1], realslp[2])


        tmp_pose = PoseStamped()
        tmp_pose.header.frame_id = input_frame
        tmp_pose.pose = posemath.toMsg(input_g1)
        sm.userdata.g1 = copy.deepcopy(tmp_pose)
        tmp_pose.pose = posemath.toMsg(input_g2)
        sm.userdata.g2 = copy.deepcopy(tmp_pose)
        tmp_pose.pose = posemath.toMsg(input_p1)
        sm.userdata.p1 = copy.deepcopy(tmp_pose)
        tmp_pose.pose = posemath.toMsg(input_p2)
        sm.userdata.p2 = copy.deepcopy(tmp_pose)

        sm.userdata.ik_link_1 = 'r1_ee'
        sm.userdata.ik_link_2 = 'r2_ee'
        sm.userdata.table_id = 't2'
        sm.userdata.offset_plus = 0.00
        sm.userdata.offset_minus = 0.1
        sm.userdata.rotation_angle_1 = 0
        sm.userdata.rotation_angle_2 = 0
        table_offset = 0.075
        
        
        
        
        br = tf.TransformBroadcaster()
        rospy.sleep(1.0)
        tmp = posemath.fromMsg(sm.userdata.g1)
        br.sendTransform(tmp.p, tmp.M.GetQuaternion() , rospy.Time.now(), 'G11' , ud.g1.header.frame_id)
        
        """
        sm_go_home = gensm_plan_vis_exec(PlanToHomeState(), output_keys=['trajectory']);
        with sm:
            smach.Sequence.add('GFOLD_FINAL', GFold2State(True, True, table_offset, 0.01))

        outcome = sm.execute()
"""
        #raw_input("Hit key to continue")

    ##  Load an image grabed by a camera.
    #
    #   Images are now stored on a local HDD
    #   @param index The index of image to be loaded
    #   @return The image loaded from a file        
    def getImageOfObsObject(self, index):
        logging.debug("TAKE_PICTURE - Begin")
        
        takenImage = None
        self.lastImageIndex = index
        logging.info("Get image - waiting for msg")
        pcData = rospy.wait_for_message("/xtion2/depth_registered/points", PointCloud2)
        
        #convert it to format accepted by openCV
        logging.info("Get image - converting")
        arr = pointclouds.pointcloud2_to_array(pcData, split_rgb=self.has_rgb(pcData))

        if arr == None:
            rospy.logerr(rospy.get_name() + ": Failed to convert PoinCloud to numpy array!")
            return False

        if self.has_fields(['x', 'y', 'z'], arr):
            xyz = np.zeros(list(arr.shape) + [3], dtype=np.float32)
            xyz[..., 0] = arr['x']
            xyz[..., 1] = arr['y']
            xyz[..., 2] = arr['z']
            self.pcMap = xyz

        if self.has_fields(['r', 'g', 'b'], arr):
            rgb = np.zeros(list(arr.shape) + [3], dtype=np.uint8)
            rgb[..., 0] = arr['b']
            rgb[..., 1] = arr['g']
            rgb[..., 2] = arr['r']
            
        takenImage = cv.fromarray(rgb)
        
        cv.NamedWindow("Image from Kinect")
        cv.ShowImage("Image from Kinect", takenImage)
        cv.WaitKey()
        cv.DestroyWindow("Image from Kinect")

        logging.debug("TAKE_PICTURE - End")
        return takenImage
        
    ## Compute and return homography between side and top view
    #
    #  It takes the current view into one directly above the table. Correspondence 
    #    between points was made by a hand.
    #  @return 3x3 homography matrix
    def get_homography(self):
        H = cv.CreateMat(3, 3, cv.CV_32FC1)
        cv.Set(H, 0.0)
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1
        return H 
        
        
# Private functions
    def has_rgb(self, msg):
        return "rgb" in [pf.name for pf in msg.fields]
        
    def has_fields(self, fields, arr):
        for field in fields:
            if not field in arr.dtype.names:
                return False

        return True
        
    def checkAndReplaceNaN(self, point, xPos, xBoundary, yPos, yBoundary):
        print "Start index : " + str([xPos, yPos])
        xlb = int(xPos - xBoundary)
        xrb = int(xPos + xBoundary)
        ylb = int(yPos - yBoundary)
        yrb = int(yPos + yBoundary)
        it = 0
        if(math.isnan(point[0])):
            for x in range (xlb, xrb):
               for y in range (ylb, yrb):
                   it += 1
                   point = self.pcMap[x, y]
                   if not math.isnan(point[0]) : break
               if not math.isnan(point[0]) : break
        if(math.isnan(point[0])): print "Stoji to za hovno " + str(it)
        return point

