#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""The Video Widget example shows how to implement a video widget
using QtMultimedia's QAbstractVideoSurface.

The following is a translation into PyQt5 from the C++ example found in
C:\QtEnterprise\5.1.1\msvc2010\examples\multimediawidgets\customvideosurface\customvideowidget."""
#from __future__ import unicode_literals

import csv
import yaml
import cv2
import os
import rosbag
import argparse
import textwrap
import rospy
import json
import random
import matplotlib
import math
from operator import itemgetter
import itertools

matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *
import warnings

from matplotlib.widgets import Cursor
from numpy import arange, sin, pi
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.transforms as transforms
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

programmName = os.path.basename(sys.argv[0])

#input parameters
def parse_arguments():
    inputFile = sys.argv[-1]
    #print inputFile 
    return inputFile

def buffer_data(bag, input_topic, compressed):
    image_buff = []
    time_buff  = []
    start_time = None
    depthData = []
    bridge     = CvBridge()
    #bag = rosbag.Bag(bagFile)
    #Buffer the images, timestamps from the rosbag
    for topic, msg, t in bag.read_messages(topics=[input_topic]):
        depthData+=msg.data
        if start_time is None:
            start_time = t

        #Get the image
        if not compressed:
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "32FC1")
            except CvBridgeError as e:
                print e
        else:
            nparr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)

        # normalize depth image 0 to 255
        depthImg = np.array(cv_image, dtype=np.float32)
        cv2.normalize(depthImg, depthImg, 0, 255, cv2.NORM_MINMAX)
        time_buff.append(t.to_sec() - start_time.to_sec())
        image_buff.append(depthImg)

    return image_buff, time_buff  


def depth_bag_file(bagFile):
    info_dict = yaml.load(bagFile._get_yaml_info())
    topics = info_dict['topics']
    topic = topics[1]
    duration = info_dict['duration']
    topic_type = topic['type']
    message_count = topic['messages']

    #Messages for test
    print "\nRosbag topics found: "
    for top in topics:
        print "\t- ", top["topic"], "\n\t\t-Type: ", topic["type"],"\n\t\t-Fps: ", topic["frequency"]

    #Checking if the topic is compressed
    if 'CompressedImage' in topic_type:
        compressed = True
    else:
        compressed = False

    #Get framerate
    framerate = message_count/duration

    return message_count,duration,compressed, framerate


def runMain(bagFileName):
    (message_count,duration,compressed, framerate) = depth_bag_file(bagFileName)
    (imageBuffer, time_buff) = buffer_data(bagFileName, "/camera/depth/image_raw", compressed)
    (major, _, _) = cv2.__version__.split(".")
    if major == '3':
    	fourcc = cv2.VideoWriter_fourcc('X', 'V' ,'I', 'D')
    else:
    	fourcc = cv2.cv.CV_FOURCC('X', 'V' ,'I', 'D')
    height, width = imageBuffer[0].shape
    # 0 for grayscale image 
    # non zero values for color frames
    video_writer = cv2.VideoWriter("myvidDepth.avi", fourcc, framerate, (width,height), 0)

    if not video_writer.isOpened():
        self.errorMessages(2)
    else:
        #VideoWriter(const string& filename, int fourcc, double fps, Size frameSize, bool isColor=true)
        for frame in imageBuffer:
            depthFrame = frame.astype('uint8')
            #print type(frame)
            video_writer.write(depthFrame)
        video_writer.release()


