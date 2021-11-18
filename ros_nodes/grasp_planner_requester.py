#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import logging
import numpy as np
import os
import rosgraph.roslogging as rl
import rospy
import sys
import yaml
import rospkg
import cv2
from cv_bridge import CvBridge, CvBridgeError

from autolab_core import Point, Logger
from autolab_core import BinaryImage, CameraIntrinsics, ColorImage, DepthImage
from visualization import Visualizer2D as vis

from gqcnn.grasping import Grasp2D, SuctionPoint2D, GraspAction
from gqcnn.msg import GQCNNGrasp
from gqcnn.srv import GQCNNGraspPlanner, GQCNNGraspPlannerSegmask, GQCNNGraspPlannerRequest

from sensor_msgs.msg import CameraInfo, Image

import matplotlib
matplotlib.use('Qt5Agg')


# Set up logger.
logger = Logger.get_logger("ros_nodes/grasp_planner_requester.py")


class GQCNN():

    def __init__(self):

        rospy.init_node("gqcnn_policy")
        rospy.loginfo("Starting gqcnn_policy node")

        package_path = rospkg.RosPack().get_path('gqcnn')
        with open(package_path + '/configs/default.yaml') as f:
            self.cfg = yaml.safe_load(f)

        # assume that rs = 640 x 480
        camera_info = rospy.wait_for_message(self.cfg["camera_info"], CameraInfo)
        K = np.array(camera_info.K)
        camera_intr = CameraIntrinsics(fx=K[0], fy=K[4], cx=K[2], cy=K[5], height=640, width=640, frame="/gr/rs2/")
        self.camera_intr = camera_intr.resize(0.8) # 640 x 480 to 512 x 384
        self.cv_bridge = CvBridge()


        # Wait for grasp planning service and create service proxy.
        rospy.wait_for_service("/gqcnn/grasp_planner")
        rospy.wait_for_service("/gqcnn/grasp_planner_segmask")
        self.plan_grasp = rospy.ServiceProxy("/gqcnn/grasp_planner", GQCNNGraspPlanner)
        self.plan_grasp_segmask = rospy.ServiceProxy("gqcnn/grasp_planner_segmask", GQCNNGraspPlannerSegmask)

        request_plan_grasp = rospy.Service('/gqcnn/grasp_planner_request', GQCNNGraspPlannerRequest, self.request_plan_grasp)
        self.grasp_vis_publisher = rospy.Publisher("/gqcnn/grasp_plan_vis", Image, queue_size=1)
        rospy.loginfo("Ready to request GQCNN grasp planning")
        rospy.loginfo("Grasp planning results will be published at /gqcnn/grasp_plan_vis")
    
    def request_plan_grasp(self, msg):

        # get rgb and depth
        color = rospy.wait_for_message(self.cfg["color"], Image)
        color = self.cv_bridge.imgmsg_to_cv2(color, desired_encoding='bgr8')
        color = cv2.resize(color, (512, 384))
        color_im = ColorImage(color, frame=self.camera_intr.frame)

        depth = rospy.wait_for_message(self.cfg["depth"], Image)
        depth = self.cv_bridge.imgmsg_to_cv2(depth, desired_encoding='32FC1')
        depth = depth / 1000
        depth = cv2.resize(depth, (512, 384))
        depth_im = DepthImage(depth, frame=self.camera_intr.frame)
        depth_im = depth_im.inpaint(rescale_factor=self.cfg["inpaint_rescale_factor"])
        

        # !TODO: support segmentation mask of UOAIS
        segm_mask = depth_im.invalid_pixel_mask().inverse()
        # segmask = BinaryImage.open(segmask_filename, frame=camera_intr.frame)
        # grasp_resp = self.plan_grasp_segmask(color_im.rosmsg, depth_im.rosmsg,
        #                                 camera_intr.rosmsg, segmask.rosmsg)
        grasp_resp = self.plan_grasp(color_im.rosmsg, depth_im.rosmsg, self.camera_intr.rosmsg)
        grasp = grasp_resp.grasp

        # Convert to a grasp action.
        grasp_type = grasp.grasp_type
        if grasp_type == GQCNNGrasp.PARALLEL_JAW:
            center = Point(np.array([grasp.center_px[0], grasp.center_px[1]]),
                        frame=self.camera_intr.frame)
            grasp_2d = Grasp2D(center,
                            grasp.angle,
                            grasp.depth,
                            width=self.cfg["gripper_width"],
                            camera_intr=self.camera_intr)
        elif grasp_type == GQCNNGrasp.SUCTION:
            center = Point(np.array([grasp.center_px[0], grasp.center_px[1]]),
                        frame=self.camera_intr.frame)
            grasp_2d = SuctionPoint2D(center,
                                    np.array([0, 0, 1]),
                                    grasp.depth,
                                    camera_intr=self.camera_intr)
        else:
            raise ValueError("Grasp type %d not recognized!" % (grasp_type))
        try:
            thumbnail = DepthImage(self.cv_bridge.imgmsg_to_cv2(
                grasp.thumbnail, desired_encoding="passthrough"),
                                frame=self.camera_intr.frame)
        except CvBridgeError as e:
            logger.error(e)
            logger.error("Failed to convert image")
            sys.exit(1)
        action = GraspAction(grasp_2d, grasp.q_value, thumbnail)

        # Vis final grasp.
        fig = vis.figure(size=(10, 10))
        vis.imshow(depth_im, vmin=0.25, vmax=0.6)
        vis.grasp(action.grasp, scale=2.5, show_center=False, show_axis=True)
        vis.title("Planned grasp on depth (Q=%.3f)" % (action.q_value))
        fig.canvas.draw()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        img = self.cv_bridge.cv2_to_imgmsg(img)
        self.grasp_vis_publisher.publish(img)
        return grasp


if __name__ == "__main__":
    
    gqcnn = GQCNN()
    rospy.spin()



