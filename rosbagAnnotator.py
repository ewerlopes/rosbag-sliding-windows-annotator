#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""The Video Widget example shows how to implement a video widget
using QtMultimedia's QAbstractVideoSurface.
The following is a translation into PyQt5 from the C++ example found in
C:\QtEnterprise\5.1.1\msvc2010\examples\multimediawidgets\customvideosurface\customvideowidget."""

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
import itertools
import ast

from matplotlib.widgets import Cursor
from numpy import arange, sin, pi
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.transforms as transforms
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm


''''''''''''''''''''''''''''''''''''
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QFont, QPainter
from PyQt5.QtCore import Qt, QUrl, pyqtSignal, QFile, QIODevice, QObject, QRect
from PyQt5.QtMultimedia import (QMediaContent,
        QMediaMetaData, QMediaPlayer, QMediaPlaylist, QAudioOutput, QAudioFormat)
from PyQt5.QtWidgets import (QApplication, QComboBox, QHBoxLayout, QPushButton,
        QSizePolicy, QVBoxLayout, QWidget, QToolTip, QLabel, QFrame, QGridLayout, QMenu, qApp, QLineEdit)

from audioGlobals import audioGlobals
import visualizeAudio as vA
import ganttChartAudio as gA
from graphicalInterfaceAudio import ApplicationWindow
import rosbagAudio
import laserGui
import rosbagDepth
''''''''''''''''''''''''''''''''''''

start_point = False
end_point = False
boxInitialized = False
annotationColors = ['#00FF00', '#FF00FF','#FFFF00','#00FFFF','#FFA500','#C0C0C0','#000000','#EAEAEA']
eventColors = ['#9fbf1f','#087649','#0a5b75','#181a8d','#7969b0','#76a9ea','#bef36e','#edfa84','#f18ed2','#753e20']
gantEnabled = False
posSlider = 0
durationSlider = 0
#Declare the basic topics for the topic box
BasicTopics = ['Audio', 'Depth', 'Video' , 'Laser']

def buffer_data(bag, input_topic, compressed):
    image_buff = []
    time_buff  = []
    start_time = None
    bridge     = CvBridge()

    #Buffer the images, timestamps from the rosbag
    for topic, msg, t in bag.read_messages(topics=[input_topic]):
        if start_time is None:
            start_time = t

        #Get the image
        if not compressed:
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print e
        else:
            nparr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)

        image_buff.append(cv_image)
        time_buff.append(t.to_sec() - start_time.to_sec())

    return image_buff,  time_buff

#Returns a buffer with boxes
def buffer_csv(csv_file):
    box_buff   = []
    metrics = []
    box_buff_action = []

    if csv_file is not None and os.path.exists(csv_file):
        with open(csv_file, 'r') as file_obj:
            csv_reader = csv.reader(file_obj, delimiter = '\t')
            row_1 = next(csv_reader)
            try:
                index = [x.strip() for x in row_1].index('Rect_id')
                if 'Class' not in row_1:
                    for row in csv_reader:
                        (rec_id,x, y, width, height) = map(int, row[index:index + 5])
                        (meter_X,meter_Y,meter_Z,top,meter_h,distance) = map(float, row[(index+5)::])
                        box_buff.append((rec_id,x, y, width, height))
                        metrics.append((meter_X,meter_Y,meter_Z,top,meter_h,distance))
                else:
                    for row in csv_reader:
                        (rec_id,x, y, width, height) = map(int, row[index:index + 5])
                        (meter_X,meter_Y,meter_Z,top,meter_h,distance) = map(float, row[(index+6)::])
                        box_buff.append((rec_id,x, y, width, height))
                        if  isinstance(row[index+5],str):
                            string = row[index+5]
                            if string.startswith('[') and string.endswith(']'):
                                #Transform a string of list to list
                                string = ast.literal_eval(string)
                                box_buff_action.append(string)
                            else:
                                box_buff_action.append(string)
                        else:
                            box_buff_action.append(row[index+5])
                        metrics.append((meter_X,meter_Y,meter_Z,top,meter_h,distance))
            except:
                return False,False,False
            return box_buff,metrics,box_buff_action
    else:
        return False,False,False

def get_bag_metadata(bag):
    info_dict       = yaml.load(bag._get_yaml_info())
    topics             = info_dict['topics']
    topic            = topics[1]
    duration       = info_dict['duration']
    topic_type       = topic['type']
    message_count = topic['messages']
    topics_List = []
    #Messages for test
    print "\nRosbag topics found: "
    for top in topics:
        print "\t- ", top["topic"], "\n\t\t-Type: ", topic["type"],"\n\t\t-Fps: ", topic["frequency"]
        topics_List.append(top["topic"])
    topics_List = sorted(set(topics_List))
    print topics_List
    #Checking if the topic is compressed
    if 'CompressedImage' in topic_type:
        compressed = True
    else:
        compressed = False

    #Get framerate
    framerate = message_count/duration

    return message_count,duration,compressed, framerate,topics_List


class VideoWidgetSurface(QAbstractVideoSurface):

    def __init__(self, widget, parent=None):
        super(VideoWidgetSurface, self).__init__(parent)
        self.widget = widget
        self.imageFormat = QImage.Format_Invalid
        global frameCounter
        frameCounter = 0 #Frame Counter initialize

    def supportedPixelFormats(self, handleType=QAbstractVideoBuffer.NoHandle):
        formats = [QVideoFrame.PixelFormat()]
        if (handleType == QAbstractVideoBuffer.NoHandle):
            for f in [QVideoFrame.Format_RGB32, QVideoFrame.Format_ARGB32, QVideoFrame.Format_ARGB32_Premultiplied, QVideoFrame.Format_RGB565, QVideoFrame.Format_RGB555,QVideoFrame.Format_BGR24,QVideoFrame.Format_RGB24]:
                formats.append(f)
        return formats

    def isFormatSupported(self, _format):
        imageFormat = QVideoFrame.imageFormatFromPixelFormat(_format.pixelFormat())
        size = _format.frameSize()
        _bool = False
        if (imageFormat != QImage.Format_Invalid and not size.isEmpty() and _format.handleType() == QAbstractVideoBuffer.NoHandle):
            _bool = True
        return _bool

    def start(self, _format):
        imageFormat = QVideoFrame.imageFormatFromPixelFormat(_format.pixelFormat())
        size = _format.frameSize()
        #frameCounter = 0 #Frame Counter initialize
        if (imageFormat != QImage.Format_Invalid and not size.isEmpty()):
            self.imageFormat = imageFormat
            self.imageSize = size
            self.sourceRect = _format.viewport()
            QAbstractVideoSurface.start(self, _format)
            self.widget.updateGeometry()
            self.updateVideoRect()
            return True
        else:
            return False

    def stop(self):

        self.currentFrame = QVideoFrame()
        self.targetRect = QRect()
        QAbstractVideoSurface.stop(self)

        self.widget.update()

    def present(self, frame):
        global frameCounter,removeBool
        if (self.surfaceFormat().pixelFormat() != frame.pixelFormat() or self.surfaceFormat().frameSize() != frame.size()):
            self.setError(QAbstractVideoSurface.IncorrectFormatError)
            self.stop()
            return False
        else:
            self.currentFrame = frame
            frameCounter += 1
            removeBool = True #Removes the boxes on current frame
            self.widget.repaint(self.targetRect)
            return True

    def videoRect(self):
        return self.targetRect

    def updateVideoRect(self):
        size = self.surfaceFormat().sizeHint()
        size.scale(self.widget.size().boundedTo(size), Qt.KeepAspectRatio)
        self.targetRect = QRect(QPoint(0, 0), size);
        self.targetRect.moveCenter(self.widget.rect().center())

    def paint(self, painter):
        if (self.currentFrame.map(QAbstractVideoBuffer.ReadOnly)):
            oldTransform = painter.transform()
            if (self.surfaceFormat().scanLineDirection() == QVideoSurfaceFormat.BottomToTop):
                painter.scale(1, -1);
                painter.translate(0, -self.widget.height())

            image = QImage(self.currentFrame.bits(),
                    self.currentFrame.width(),
                    self.currentFrame.height(),
                    self.currentFrame.bytesPerLine(),
                    self.imageFormat
            )

            painter.drawImage(self.targetRect, image, self.sourceRect)
            painter.setTransform(oldTransform)

            self.currentFrame.unmap()

class VideoWidget(QWidget):

    def __init__(self, parent=None):
        global classLabels, imageBuffer
        super(VideoWidget, self).__init__(parent)
        self.setAutoFillBackground(False)
        self.setAttribute(Qt.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WA_OpaquePaintEvent)
        palette = self.palette()
        palette.setColor(QPalette.Background, Qt.black)
        self.setPalette(palette)
        self.setSizePolicy(QSizePolicy.MinimumExpanding ,
        QSizePolicy.MinimumExpanding)
        self.surface = VideoWidgetSurface(self)
        self.vanishBox = False
        self.enableWriteBox = False
        self.annotEnabled = False
        self.annotClass = 'Clear'
        self.deleteEnabled = False
        self.deleteAllBoxes = False
        self.buttonLabels = []
        self.addEventLabels = []
        self.stopEventLabels = []
        classLabels = []
        highLabels = []
        imageBuffer = []



    def videoSurface(self):
        return self.surface

    #Shows the right click menu
    def contextMenuEvent(self,event):
        global posX
        global posY
        global classLabels,gantChart,gantEnabled,highLabels,frameCounter
        self.stopEventEnabled = False
        self.addEventEnabled = False

        if event.reason() == QContextMenuEvent.Mouse:
            menu = QMenu(self)
            clear = menu.addAction('Clear')

            posX = event.pos().x()
            posY = event.pos().y()

            for i in classLabels:
                self.buttonLabels.append(menu.addAction(i))

            deleteBox = menu.addAction('Delete Box')
            deleteAllBoxes = menu.addAction('Delete All Boxes')
            addEvent = menu.addMenu('Add Event')
            stopEvent = menu.addMenu('Stop Event')
            stopEvent.setEnabled(False)
            #Show only annotated high classes of the box
            for i in range(len(player.videobox[frameCounter].box_Id)):
                x,y,w,h = player.videobox[frameCounter].box_Param[i]
                if posX > x and posX < (x+w) and posY > y and posY < (y+h):
                    box_id = player.videobox[frameCounter].box_Id.index(i)
                    if len(player.videobox[frameCounter].annotation) > 0:
                        for annot in player.videobox[frameCounter].annotation[box_id]:
                            if annot in highLabels and annot not in self.checkStopEventMenu:
                                self.checkStopEventMenu.append(annot)
                                self.stopEventLabels.append(stopEvent.addAction(annot))
                                stopEvent.setEnabled(True)
            #Initiate add Event menu
            for label in highLabels:
                self.addEventLabels.append(addEvent.addAction(label))
            changeId = menu.addAction('Change Id')
            cancel = menu.addAction('Cancel')
            action = menu.exec_(self.mapToGlobal(event.pos()))

            #Check which submenu clicked
            if action is not None:
                if action.parent() == addEvent:
                    self.addEventEnabled = True
                elif action.parent() == stopEvent:
                    self.stopEventEnabled = True

            if self.addEventEnabled:
                for index, key in enumerate(self.addEventLabels):
                    if action == key:
                        self.annotClass = highLabels[index]
                        self.annotEnabled = True
                        self.addEventEnabled = False

            elif self.stopEventEnabled:
                for index, key in enumerate(self.stopEventLabels):
                    if action == key:
                        player.videobox[frameCounter].removeEvent(box_id,self.stopEventLabels[index].text() )
                        self.stopEventEnabled = False

            for i,key in enumerate(self.buttonLabels):
                if action == key:
                    self.annotClass = classLabels[i]
                    self.annotEnabled = True
            if action == deleteBox:
                self.deleteEnabled = True
            elif action ==  deleteAllBoxes:
                self.deleteAllBoxes = True
            elif action == changeId:
                #Call the textbox
                self.newBoxId = textBox()
                self.newBoxId.setGeometry(QRect(500, 100, 300, 100))
                self.newBoxId.show()
            elif action == cancel:
                pass
            elif action == clear:
                self.annotClass = 'Clear'
                self.annotEnabled = True

            '''
            posX = event.pos().x()
            posY = event.pos().y()
            '''
            self.repaint()
            self.buttonLabels = []
            self.addEventLabels = []
            self.stopEventLabels = []
            self.checkStopEventMenu = []
        self.annotEnabled = False
        self.stopEventEnabled = False

        #print player.videobox[frameCounter].annotation
        #gantEnabled = True
        gantChart.axes.clear()
        gantChart.drawChart()
        gantChart.draw()

    def sizeHint(self):
        return self.surface.surfaceFormat().sizeHint()

    #Shows the video and bound boxes on it
    def paintEvent(self, event):
        global start_point
        global end_point
        global frameCounter
        global timeId

        painter = QPainter(self)
        rectPainter = QPainter(self)
        boxIdPainter = QPainter()

        if not rectPainter.isActive():
            rectPainter.begin(self)

        if (self.surface.isActive()):
            videoRect = QRegion(self.surface.videoRect())
            if not videoRect.contains(event.rect()):
                region = event.region()
                region.subtracted(videoRect)
                brush = self.palette().background()
                for rect in region.rects():
                    painter.fillRect(rect, brush)
            self.surface.paint(painter)
        else:
            painter.fillRect(event.rect(), self.palette().window())
        '''
        #If you press control and click, remove the clicked box from the list
        if player.controlEnabled :
            posX = self.eraseRectPos.x()
            posY = self.eraseRectPos.y()
            for i in range(len(player.videobox[frameCounter].box_Id)):
                x,y,w,h = player.videobox[frameCounter].box_Param[i]
                if posX > x and posX < (x+w) and posY > y and posY < (y+h):
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(Qt.red)
                    rectPainter.drawRect(x,y,w,h)
                    timeId = player.videobox[frameCounter].timestamp[0]
                    player.videobox[frameCounter].removeBox() #CTRL + CLICK removes the box
        '''

        if self.deleteEnabled:
            i = 0
            while i < len(player.videobox[frameCounter].box_Id):
                x,y,w,h = player.videobox[frameCounter].box_Param[i]
                if posX > x and posX < (x+w) and posY > y and posY < (y+h):
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(Qt.red)
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()
                    timeId = player.videobox[frameCounter].timestamp[0]
                    player.videobox[frameCounter].removeSpecBox(player.videobox[frameCounter].box_Id[i])
                    for j in range(len(player.videobox[frameCounter].box_Id)):
                        x,y,w,h = player.videobox[frameCounter].box_Param[j]
                        if not rectPainter.isActive():
                            rectPainter.begin(self)
                        rectPainter.setRenderHint(QPainter.Antialiasing)
                        rectPainter.setPen(QColor(self.getColorBox(player.videobox[frameCounter].annotation[j])))
                        rectPainter.drawRect(x,y,w,h)
                        rectPainter.end()

                        if not boxIdPainter.isActive():
                            boxIdPainter.begin(self)
                        boxIdPainter.setRenderHint(QPainter.Antialiasing)
                        boxIdPainter.setPen(QColor(255,0,0))
                        boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[j]))
                        boxIdPainter.end()
                i += 1
            self.deleteEnabled = False
        #Deletes all boxes in current framerate
        elif self.deleteAllBoxes:
            timeId = player.videobox[frameCounter].timestamp[0]
            for i in range(len(player.videobox[frameCounter].box_Id)):
                    x,y,w,h = player.videobox[frameCounter].box_Param[i]
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setPen(Qt.red)
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()
            player.videobox[frameCounter].removeAllBox()
            self.deleteAllBoxes = False
        #Enabled when annotating
        elif self.annotEnabled:
            self.frameNumber = frameCounter
            box = None
            for i in range(len(player.videobox[frameCounter].box_Id)):
                x,y,w,h = player.videobox[frameCounter].box_Param[i]
                if posX > x and posX < (x+w) and posY > y and posY < (y+h):
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    if self.annotClass in highLabels:
                        rectPainter.setPen(QColor(self.getColorBox(player.videobox[frameCounter].annotation[i])))
                    else:
                        rectPainter.setPen(QColor(self.getColorBox(self.annotClass)))
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    #Paint box id in current frame
                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()
                    player.videobox[frameCounter].changeClass(i,str(self.annotClass))
                    box = i
                else:
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(QColor(self.getColorBox(player.videobox[frameCounter].annotation[i])))
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    #Paint box id in current frame
                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()

            #Annotate the box at remaining frames
            while self.frameNumber < len(player.time_buff):
                if box >= len(player.videobox[self.frameNumber].box_Id) or box is None:
                    self.frameNumber += 1
                    continue
                player.videobox[self.frameNumber].changeClass(box,str(self.annotClass))
                self.frameNumber += 1

        elif start_point is True and end_point is True:
                x = event.rect().x()
                y = event.rect().y()
                w = event.rect().width()
                h = event.rect().height()

                #Keep the timestamp to add the new box
                if  len(player.videobox[frameCounter].timestamp):
                    timeId = player.videobox[frameCounter].timestamp[0]

                if self.enableWriteBox:
                    boxNumber = len(player.videobox[frameCounter].box_Id)
                    #If id already in the list then give the next id
                    if boxNumber in player.videobox[frameCounter].box_Id:
                        boxNumber += 1
                    player.videobox[frameCounter].addBox(timeId,[boxNumber,x,y,w,h],['Clear'])
                    self.enableWriteBox = False

                for i in range(len(player.videobox[frameCounter].box_Id)):
                    x,y,w,h = player.videobox[frameCounter].box_Param[i]
                    if not rectPainter.isActive():
                            rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(QColor(self.getColorBox(player.videobox[frameCounter].annotation[i])))
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()

        #Play the bound boxes from csv
        elif len(player.videobox) > 0 and frameCounter < len(player.time_buff):
                for i in range(len(player.videobox[frameCounter].box_Id)):
                    x,y,w,h = player.videobox[frameCounter].box_Param[i]
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(QColor(self.getColorBox(player.videobox[frameCounter].annotation[i])))
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()

        if rectPainter.isActive():
            rectPainter.end()

    #Mouse callback handling Boxes
    def mousePressEvent(self,event):
        global start_point,end_point

        if player.controlEnabled and QMouseEvent.button(event) == Qt.LeftButton:
             self.eraseRectPos= QMouseEvent.pos(event)
             self.repaint()
        elif QMouseEvent.button(event) == Qt.LeftButton:
            if start_point is True and end_point is True:
                pass
            elif start_point is False:
                QPoint.pos1 = QMouseEvent.pos(event)
                start_point = True
            elif end_point is False:
                QPoint.pos2 = QMouseEvent.pos(event)
                rect = QRect(QPoint.pos1,QPoint.pos2)
                end_point = True
                self.repaint()
                self.enableWriteBox = True
                self.repaint(rect)

                start_point = False
                end_point = False

    def resizeEvent(self, event):
        QWidget.resizeEvent(self, event)
        self.surface.updateVideoRect()

    def getColorBox(self,action):
        global classLabels,highLabels
        for label in action:
            if label in classLabels:
                color = label
                return annotationColors[classLabels.index(label) % len(annotationColors)]
            elif label == 'Clear':
                color = 'Clear'
                return '#0000FF'
            elif label in highLabels:
                pass

        if action in classLabels:
            for index,key in enumerate(classLabels):
                if action == key:
                    return annotationColors[index % len(annotationColors)]
                elif action == 'Clear':
                    return '#0000FF'
        else:
            for index,key in enumerate(player.videobox[frameCounter].annotation):
                if key in classLabels:
                    return annotationColors[classLabels.index(key) % len(annotationColors)]
                elif key == 'Clear':
                    return '#0000FF'

class textBox(QWidget):

    def __init__(self):
        global frameCounter
        global posX,posY

        QWidget.__init__(self)
        self.setWindowTitle('Set Box id')
        self.main_widget = QWidget(self)
        self.boxId = QLineEdit(self)
        self.Ok = QPushButton("Ok", self)

    def paintEvent(self, event):
        self.boxId.setPlaceholderText('Box Id:')
        self.boxId.setMinimumWidth(100)
        self.boxId.setEnabled(True)

        self.boxId.move(90, 15)
        self.Ok.move(115, 60)

        self.boxId.textChanged.connect(self.boxChanged)
        self.Ok.clicked.connect(self.closeTextBox)

        self.Ok.show()
        self.boxId.show()

    def boxChanged(self,text):
        self.box_Idx = text

    def closeTextBox(self):
        try:
            self.box_Idx = int(self.box_Idx)
        except:
            msgBox = QMessageBox()
            msgBox.setText("Wrong type, integer expected")
            msgBox.resize(100,40)
            msgBox.exec_()

        #Check id
        for i in range(len(player.videobox[frameCounter].box_Id)):
            if self.box_Idx == player.videobox[frameCounter].box_Id[i]:
                #Box Id already given
                msgBox = QMessageBox()
                msgBox.setText("Box Id already given")
                msgBox.resize(100,40)
                msgBox.exec_()

        for i in range(len(player.videobox[frameCounter].box_Id)):
            x,y,w,h = player.videobox[frameCounter].box_Param[i]
            if posX > x and posX  < (x+w) and posY > y and posY < (y+h):
                old_value = player.videobox[frameCounter].box_Id[i]
                player.videobox[frameCounter].box_Id[i] = self.box_Idx
                self.writeEnable = True
                self.frameNumber = frameCounter
                old_index = i
                break

        if self.writeEnable:
            while self.frameNumber < len(player.time_buff):
                if old_value in player.videobox[self.frameNumber].box_Id:
                    player.videobox[self.frameNumber].box_Id[old_index] = self.box_Idx
                self.frameNumber += 1
            self.writeEnable = False

        self.Ok.clicked.disconnect()
        self.close()

#Class for Drop down boxes about topic selection
class TopicBox(QDialog):
    def __init__(self):
        super(TopicBox,self).__init__()
        global BasicTopics,Topics
        self.setWindowTitle('Select Topics')
        self.setGeometry(280, 260, 440, 400)
        self.move(QApplication.desktop().screen().rect().center()- self.rect().center())
        self.okButton = QPushButton("Ok", self)
        self.okButton.move(180,360)
        self.okButton.clicked.connect(self.close_window)
        self.okButton.setEnabled(False)

        self.okButtonPush = False
        self.topic_options = []

    def show_topics(self):
        x = 30
        y = 40
        self.dropDownBox = []
        self.temp_topics = []

        for index,topic in enumerate(BasicTopics):
            self.topic_options.append(QLabel(self))
            self.topic_options[index].move(x,y)
            self.topic_options[index].setText(BasicTopics[index])
            self.dropDownBox.append(QComboBox(self))
            y += 60

        x = 120
        y = 35
        for key,option in enumerate(self.dropDownBox):
            self.dropDownBox[key].addItem('Choose Topic')
            self.dropDownBox[key].addItems(Topics)
            self.dropDownBox[key].move(x, y)
            self.dropDownBox[key].currentTextChanged.connect(self.selectionchange)
            y += 60

        self.show()

    def selectionchange(self,text):
        topic_counter = 0
        for key,option in enumerate(self.dropDownBox):
            if self.dropDownBox[key].currentText() == 'Choose Topic':
                topic_counter += 1
        if topic_counter == len(BasicTopics):
            self.okButton.setEnabled(False)
        else:
            self.okButton.setEnabled(True)

        for key,option in enumerate(self.dropDownBox):
            if text == self.dropDownBox[key].currentText():
                ddbox_index = key

        if len(self.temp_topics) > 0:
            for idx,value in enumerate(self.temp_topics):
                if value[0] == ddbox_index:
                    self.temp_topics.pop(idx)
                    self.temp_topics.append([ddbox_index,text])
            if [ddbox_index,text] not in self.temp_topics:
                self.temp_topics.append([ddbox_index,text])
        else:
            self.temp_topics.append([ddbox_index,text])

    def close_window(self):
        #Sort by its first element
        self.temp_topics.sort(key=lambda x: x[0])
        self.okButtonPush = True
        self.close()

    def closeEvent(self,event):
        if not self.okButtonPush:
            msgBox = QMessageBox()
            msgBox.setIcon(msgBox.Question)
            msgBox.setText("Program will exit, are you sure?")
            msgBox.resize(140,60)
            msgBox.setStandardButtons(QMessageBox.No | QMessageBox.Yes)
            msgBox.setDefaultButton(QMessageBox.No)
            ret = msgBox.exec_()
            if ret == QMessageBox.Yes:
                self.close()
                player.close()
            elif ret == QMessageBox.No:
                event.ignore()

class VideoPlayer(QWidget):
    def __init__(self, parent=None):
        global gantChart,Topics
        super(VideoPlayer, self).__init__(parent)
        self.videobox = []
        Topics = None
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        #self.setWindowFlags(self.windowFlags() | QtCore.Qt.CustomizeWindowHint)
        self.setWindowFlags( (self.windowFlags() | Qt.CustomizeWindowHint) & ~Qt.WindowMaximizeButtonHint)

        self.box_buffer = []
        self.metric_buffer = []
        self.topic_window = TopicBox()

        self.videoWidget = VideoWidget()
        self.laserScan = laserGui.LS()

        # >> LASER
        #Define Laser Buttons
        scanLayout = QHBoxLayout()
        scanLayout.addWidget(self.laserScan)

        playButtonLaser = QPushButton("Play")
        pauseButtonLaser = QPushButton("Pause")
        prevFrameButtonLaser = QPushButton("Previous")
        nextFrameButtonLaser = QPushButton("Next")
        stopButtonLaser = QPushButton("Stop")
        le = QLineEdit(self)
        le.setDragEnabled(True)
        addButtonLaser = QPushButton('Add', self)

        classes = QComboBox()
        classes.addItem('Classes')

        buttonLayoutLaser = QHBoxLayout()
        buttonLayoutLaser.addWidget(playButtonLaser)
        buttonLayoutLaser.addWidget(pauseButtonLaser)
        buttonLayoutLaser.addWidget(prevFrameButtonLaser)
        buttonLayoutLaser.addWidget(nextFrameButtonLaser)
        buttonLayoutLaser.addWidget(stopButtonLaser)
        buttonLayoutLaser.setAlignment(Qt.AlignLeft)

        classLayout = QVBoxLayout()
        classLayout.addWidget(classes)
        classLayout.addWidget(le)
        classLayout.addWidget(addButtonLaser)
        classLayout.setAlignment(Qt.AlignTop)


        #Define Connections
        playButtonLaser.clicked.connect(self.laserPlay)
        pauseButtonLaser.clicked.connect(self.laserPause)
        prevFrameButtonLaser.clicked.connect(self.laserPrevious)
        nextFrameButtonLaser.clicked.connect(self.laserNext)
        stopButtonLaser.clicked.connect(self.laserStop)
        #annotationButtonLaser.clicked.connect(self.laserAnnotation)
        #saveButtonLaser.clicked.connect(self.laserSave)
        #classes.activated[str].connect(self.chooseClass)
        #addButton.clicked.connect(self.showObject)
        # >> END LASER

        # >> Set Fix Size at Video Widget and LaserScan
        # >> (Half Gui)
        self.laserScan.setFixedSize(640, 480)
        self.videoWidget.setFixedSize(640, 480)
        self.openButton = QPushButton("Open...")
        self.importCsv = QPushButton("Import CSV...")
        self.openButton.clicked.connect(self.openFile)
        self.importCsv.clicked.connect(self.openCsv)

        # >> most important play button
        #----------------------
        self.playButton = QPushButton()
        self.playButton.setEnabled(False)
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.play)

        # >> radio button for Depth or RGB
        #----------------------
        self.rgbButton = QRadioButton("RGB")
        self.rgbButton.setChecked(True)
        self.rgbButton.toggled.connect(self.rgbVideo)

        self.depthButton = QRadioButton("Depth")
        self.depthButton.toggled.connect(self.depth)


        self.positionSlider = QSlider(Qt.Horizontal)
        #self.positionSlider.setRange(0, audioGlobals.duration)
        self.positionSlider.setMinimum(0)
        self.positionSlider.setMaximum(audioGlobals.duration)
        self.positionSlider.sliderMoved.connect(self.setPosition)

        #add label to slider about elapsed time
        self.label_tmp = '<b><FONT SIZE=3>{}</b>'
        self.timelabel = QLabel(self.label_tmp.format('Elapsed Time: ' + str(audioGlobals.duration)))


        self.label = QHBoxLayout()
        self.label.addWidget(self.timelabel)
        self.label.setAlignment(Qt.AlignRight | Qt.AlignBottom)

        self.controlLayout = QHBoxLayout()
        self.controlLayout.addWidget(self.openButton)
        self.controlLayout.addWidget(self.importCsv)
        self.controlLayout.addWidget(self.playButton)
        self.controlLayout.addWidget(self.rgbButton)
        self.controlLayout.addWidget(self.depthButton)
        self.controlLayout.setAlignment(Qt.AlignLeft)

        self.controlEnabled = False

        videoLayout = QVBoxLayout()
        videoLayout.addWidget(self.videoWidget)
        videoLayout.addLayout(self.controlLayout)

        self.controlLaser = QHBoxLayout()
        self.controlLaser.addLayout(buttonLayoutLaser)
        self.controlLaser.addLayout(self.label)


        # >> Video Gantt Chart
        self.gantt = gantShow()
        gantChart = self.gantt
        gantChart.setFixedSize(1440, 130)


        # >> Define Audio annotations and gantt chart
        #----------------------

        self.wave = vA.Waveform()
        audioGlobals.fig = self.wave
        self.wave.axes.get_xaxis().set_visible(False)
        self.wave.draw()
        self.wave.setFixedSize(1440, 185)


        self.chart = gA.Chart()
        audioGlobals.chartFig = self.chart
        self.chart.setFixedSize(1440, 95)

        # >> Define Audio Player buttons
        #----------------------
        playButtonAudio = QPushButton("Play")
        pauseButtonAudio = QPushButton("Pause")
        stopButtonAudio = QPushButton("Stop")

        # >> Define Audio layouts
        #----------------------
        waveLayout = QVBoxLayout()
        waveLayout.addWidget(self.wave)
        waveLayout.addWidget(self.chart)

        # >> Laser Buttons Layout
        #----------------------
        buttonLayoutAudio = QHBoxLayout()
        buttonLayoutAudio.addWidget(playButtonAudio)
        buttonLayoutAudio.addWidget(pauseButtonAudio)
        buttonLayoutAudio.addWidget(stopButtonAudio)
        buttonLayoutAudio.setAlignment(Qt.AlignLeft)


        laserClass = QHBoxLayout()
        laserClass.addLayout(scanLayout)
        laserClass.addLayout(classLayout)
        laserClass.setAlignment(Qt.AlignTop)

        layoutLaser = QVBoxLayout()
        layoutLaser.addLayout(laserClass)
        layoutLaser.addLayout(self.controlLaser)
        layoutLaser.setAlignment(Qt.AlignBottom)

        # >> Specify final layout align
        #----------------------
        laserAndVideoLayout = QHBoxLayout()
        laserAndVideoLayout.addLayout(videoLayout)
        laserAndVideoLayout.addLayout(layoutLaser)

        mainLayout = QVBoxLayout()
        mainLayout.addLayout(laserAndVideoLayout)
        mainLayout.addWidget(self.positionSlider)
        mainLayout.addWidget(self.gantt)
        mainLayout.addLayout(waveLayout)
        mainLayout.addLayout(buttonLayoutAudio)

        self.setLayout(mainLayout)

        self.mediaPlayer.setVideoOutput(self.videoWidget.videoSurface())
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)

        # >> Player Buttons
        #----------------------
        playButtonAudio.clicked.connect(self.audioPlay)
        pauseButtonAudio.clicked.connect(self.audioPause)
        stopButtonAudio.clicked.connect(self.audioStop)


    def pauseMedia(self):
        self.mediaPlayer.pause()
        self.Pause()

    # VIDEO SWITCH RGB <-> Depth
    #----------------------

    def rgbVideo(self, enabled):
        if enabled:
            global frameCounter

            self. depthEnable = False
            self.rgbEnable = True
            position = self.mediaPlayer.position()
            self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath('myvid.avi'))))
            self.mediaPlayer.setPosition(position)
            self.player.setPosition(position)
            self.mediaPlayer.play()
            self.audioPlay()
            self.playButton.setEnabled(True)

    def depth(self, enabled):
        global frameCounter
        if enabled:

            self.rgbEnable = False
            self.depthEnable = True
            position = self.mediaPlayer.position()
            self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath('myvidDepth.avi'))))
            self.mediaPlayer.setPosition(position)
            self.player.setPosition(position)
            self.mediaPlayer.play()
            self.audioPlay()
            self.playButton.setEnabled(True)

    # AUDIO PLAYER BUTTON FUNCTIONS

    # >> Play audio (whole signal or segment)
    #----------------------
    def audioPlay(self):

        #GET CLICKS FROM WAVEFORM
        #----------------------
        #Initialize connection-position ONCE
        if not audioGlobals.playerStarted:
            #10ms for changePosition -> Not Delaying
            self.player.positionChanged.connect(self.checkPositionToStop)
            self.player.setNotifyInterval(10)
            if audioGlobals.durationFlag==0:
                audioGlobals.playerStarted = True
                audioGlobals.startTimeToPlay = 0
                self.start = audioGlobals.startTimeToPlay
                self.end = audioGlobals.duration*1000 - 10
                audioGlobals.endTimeToPlay = self.end
                audioGlobals.counterClick = 3
            elif audioGlobals.durationFlag==1:
                audioGlobals.playerStarted = True
                self.start = audioGlobals.startTimeToPlay
                self.end = audioGlobals.duration*1000 - 10
                audioGlobals.endTimeToPlay = self.end
                audioGlobals.counterClick = 3
            elif audioGlobals.durationFlag==2:
                audioGlobals.playerStarted = True
                self.start = audioGlobals.startTimeToPlay
                self.end = audioGlobals.endTimeToPlay
            self.player.setPosition(self.start)

        playFlag = True
        self.player.play()

    # >> Pause audio playing
    #----------------------
    def audioPause(self):
        #Not begging from self.start
        audioGlobals.playerStarted = True
        self.player.setPosition(self.time_)
        self.player.pause()

    # >> Stop audio playing
    #----------------------
    def audioStop(self):
        self.player.stop()
        #Begin again segment
        self.start = audioGlobals.startTimeToPlay
        self.player.setPosition(self.start)

    # >> Check ms in audio to stop play
    #----------------------
    def checkPositionToStop(self):
        self.time_ = self.player.position()
        #self.positionSlider.setValue(self.time_/1000)
        #print self.time_
        if self.time_ >= self.end:
            self.audioStop()
            self.player.setPosition(self.start)
            #self.positionSlider.setValue(self.start)


    # >> LASER BUTTON FUNCTIONS

    def laserPlay(self):
        global scan_widget
        scan_widget.ptime()

    def laserPause(self):
        global timer
        timer.stop()

    def laserPrevious(self):
        global cnt,ax,fw,annot,samex, samey, listofpointsx, listofpointsy,annotID
        if (cnt > 0):
            cnt = cnt-1
            ax.clear()
            ax.axis('equal')
            ax.plot(annot[cnt].samex,annot[cnt].samey, 'bo')
            if not annot[cnt].listofpointsx == []:
                for j in range(len(annot[cnt].annotID)):
                    ax.plot(annot[cnt].listofpointsx[j],annot[cnt].listofpointsy[j],color=annot[cnt].annotID[j],marker='o')
            fw.draw()
        else:
            ax.clear()
            fw.draw()

    def laserNext(self):
        global cnt,ax,fw, annot,samex, samey, listofpointsx, listofpointsy,annotID, colour_index
        if (cnt<len(annot)):
            cnt = cnt+1
            ax.clear()
            ax.axis('equal')
            colour_index = 0
            ax.plot(annot[cnt].samex,annot[cnt].samey,'bo')
            if not annot[cnt].listofpointsx == []:
                for j in range(len(annot[cnt].annotID)):
                    ax.plot(annot[cnt].listofpointsx[j],annot[cnt].listofpointsy[j],color=annot[cnt].annotID[j],marker='o')
            fw.draw()
        else:
            ax.clear()
            fw.draw()

    def laserStop(self):
        global cnt,timer,ax,fw
        cnt = 0
        timer.stop()
        ax.clear()
        fw.draw()

    def videoPosition(self):
        self.videoTime = self.mediaPlayer.position()

    def openFile(self):
        global imageBuffer,framerate,Topics
        fileName,_ = QFileDialog.getOpenFileName(self, "Open Bag", QDir.currentPath(),"(*.bag)")
        print str(fileName)

        if not fileName:
            pass
        else:
            try:
                bag = rosbag.Bag(fileName)
                rosbagAudio.runMain('rosbags/2016-07-04-16-19-14.bag')
                rosbagDepth.runMain(bag)
            except:
                self.errorMessages(0)

            #Get bag metadata
            (self.message_count,self.duration,compressed, framerate,Topics) = get_bag_metadata(bag)

            #Show the window to select topics
            self.topic_window.show_topics()

            #Buffer the rosbag, boxes, timestamps
            (imageBuffer, self.time_buff) = buffer_data(bag, "/camera/rgb/image_raw", compressed)
            #Check opencv version
            (major, _, _) = cv2.__version__.split(".")
            if major == '3':
                fourcc = cv2.VideoWriter_fourcc('X', 'V' ,'I', 'D')
            else:
                fourcc = cv2.cv.CV_FOURCC('X', 'V' ,'I', 'D')
            height, width, bytesPerComponent = imageBuffer[0].shape
            video_writer = cv2.VideoWriter("myvid.avi", fourcc, framerate, (width,height), cv2.IMREAD_COLOR)

            if not video_writer.isOpened():
                self.errorMessages(2)
            else:
                print("Video initialized")
                for frame in imageBuffer:
                    video_writer.write(frame)
                video_writer.release()

        if self.rgbButton:
            self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath('myvid.avi'))))
            self.playButton.setEnabled(True)
        elif self.depthButton:
            self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath('myvidDepth.avi'))))
            self.playButton.setEnabled(True)

        #DEFINE PLAYER-PLAYLIST
        #----------------------
        self.source = QtCore.QUrl.fromLocalFile(os.path.abspath(audioGlobals.wavFileName))
        self.content = QMediaContent(self.source)
        self.player = QMediaPlayer()
        self.playlist = QMediaPlaylist(self)
        self.playlist.addMedia(self.content)
        self.player.setPlaylist(self.playlist)

        self.wave.drawWave()
        self.wave.drawAnnotations()
        self.wave.draw()
        self.chart.drawChart()
        self.chart.draw()


    #Open CSV file
    def openCsv(self):
        global classLabels,gantEnabled,highLabels
        self.box_buffer = []
        self.metric_buffer = []

        # OPEN VIDEO - DEPTH - AUDIO
        fileName,_ =  QFileDialog.getOpenFileName(self, "Open Csv ", QDir.currentPath(),"(*.csv)")
        box_buff,metrics_buff,box_action = buffer_csv(fileName)

        if not (box_buff or metrics_buff):
            self.errorMessages(1)
        else:
            self.box_buffer = [list(elem) for elem in box_buff]
            self.metric_buffer = [list(key) for key in metrics_buff]
            #Initialize objects which are equal to frames
            self.videobox = [boundBox(count) for count in range(len(self.time_buff))]

            #Frame counter initialize
            counter = -1
            if len(box_action)>0:
                self.box_actionBuffer = [key for key in box_action]
                for idx,key in enumerate(self.box_buffer):
                    if key[0] == 0:
                        counter += 1
                        self.videobox[counter].addBox(self.time_buff[counter],key,self.box_actionBuffer[idx])
                    else:
                        self.videobox[counter].addBox(self.time_buff[counter],key,self.box_actionBuffer[idx])
            else:
                for idx,key in enumerate(self.box_buffer):
                    if key[0] == 0:
                        counter += 1
                        self.videobox[counter].addBox(self.time_buff[counter],key,['Clear'])
                    else:
                        self.videobox[counter].addBox(self.time_buff[counter],key,['Clear'])

            for box in self.videobox:
                for annot in box.annotation:
                    print 'player annot ::',annot, type(annot)


            #Parse json file
            try:
                classLabels, highLabels = self.parseJson()
            except:
                self.errorMessages(3)

            gantEnabled = True
            gantChart.axes.clear()
            gantChart.drawChart()
            gantChart.draw()

    def parseJson(self):
        json_basicLabel = []
        json_highLabel = []

        with open("labels.json") as json_file:
                json_data = json.load(json_file)
                json_label = []
                for i in json_data['basiclabels'] :
                    json_basicLabel.append(i)
                for j in json_data['highlevellabels']:
                    json_highLabel.append(j)
        return json_basicLabel,json_highLabel

    def errorMessages(self,index):
        msgBox = QMessageBox()
        if index == 0:
            msgBox.setText("Error: Incorrect Bag File")
        elif index == 1:
            msgBox.setText("Error occured: Please check CSV file")
        elif index == 2:
            msgBox.setText("Error: Video could not initialized")
        elif index == 3:
            msgBox.setText("Error: Json file path error")
        elif index == 4:
            msgBox.setText("Not integer type")
        elif index == 5:
            msgBox.setText("Box id already given")
        msgBox.resize(100,40)
        msgBox.exec_()

    def play(self):
        global frameCounter, posSlider, durationSlider
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.videoPosition()
            self.mediaPlayer.pause()
            self.audioPause()
            self.time_ = self.videoTime

        else:
            self.mediaPlayer.play()
            self.audioPlay()

        # >> Get slider position for bound box
        posSlider = self.positionSlider.value()
        #self.tickLabel.setAlignment(posSlider)
        frameCounter = int(round(self.message_count * posSlider/(self.duration * 1000)))

    def mediaStateChanged(self, state):
        if state == QMediaPlayer.PlayingState:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        self.positionSlider.setValue(position)
        self.timelabel.setText(self.label_tmp.format('Elapsed Time: ' + str(position/1000)))
        #self.timelabel.(position)

    def keyPressEvent(self,event):
        if event.key() == Qt.Key_Control:
            self.controlEnabled = True

    def keyReleaseEvent(self,event):
        if event.key() == Qt.Key_Control:
            self.controlEnabled = False

    def durationChanged(self, duration):
        global durationSlider
        durationSlider = duration
        self.positionSlider.setRange(0, duration)


    def setPosition(self, position):
        global frameCounter, posSlider
        frameCounter = int(round(self.message_count * position/(self.duration * 1000)))
        self.mediaPlayer.setPosition(position)
        posSlider = position
        self.player.setPosition(position)

    #Writes the boxes to csv
    def writeCSV(self,videobox):
        list_insert_time = []
        list_insert_box = []
        list_insert_class = []
        list_insert_param_1 = []
        list_insert_param_2 = []
        list_insert_param_3 = []
        list_insert_param_4 = []
        list_metr_param_1 = []
        list_metr_param_2 = []
        list_metr_param_3 = []
        list_metr_param_4 = []
        list_metr_param_5 = []
        list_metr_param_6 = []

        for i in self.videobox:
            for j in i.timestamp:
                list_insert_time.append(j)
            for k in i.box_Id:
                list_insert_box.append(k)
            for l in i.box_Param:
                list_insert_param_1.append(l[0])
                list_insert_param_2.append(l[1])
                list_insert_param_3.append(l[2])
                list_insert_param_4.append(l[3])
            for key in i.annotation:
                list_insert_class.append(key)

        print 'List insert::', list_insert_class

        if len(self.metric_buffer) > 0:
            for metr in self.metric_buffer:
                list_metr_param_1.append(metr[0])
                list_metr_param_2.append(metr[1])
                list_metr_param_3.append(metr[2])
                list_metr_param_4.append(metr[3])
                list_metr_param_5.append(metr[4])
                list_metr_param_6.append(metr[5])

        with open('boxes_updated.csv', 'w') as file:
            csv_writer = csv.writer(file, delimiter='\t')
            headlines = ['Timestamp','Rect_id', 'Rect_x','Rect_y','Rect_W','Rect_H','Class','Meter_X','Meter_Y','Meter_Z','Top','Height' ,'Distance']
            csv_writer.writerow(headlines)
            rows = zip(list_insert_time,list_insert_box,list_insert_param_1,list_insert_param_2,list_insert_param_3,list_insert_param_4,list_insert_class,list_metr_param_1,list_metr_param_2,list_metr_param_3,list_metr_param_4,list_metr_param_5,list_metr_param_6)
            csv_writer.writerows(rows)

    def closeEvent(self,event):
        self.writeCSV(self.videobox)

#Holds the bound box parameters
class boundBox(object):
    def __init__(self,parent=None):
        super(boundBox, self).__init__()
        self.timestamp = []
        self.box_Id = []
        self.box_Param = []
        self.annotation = []

    def addBox(self,time,key,classify):
        self.timestamp.append(time)
        self.box_Id.append(key[0])
        self.box_Param.append(key[1:])
        self.annotation.append(classify)

    def removeAllBox(self):
        self.timestamp[:] = []
        self.box_Id[:] = []
        self.box_Param[:] = []
        self.annotation[:] = []

    def removeSpecBox(self,boxid):
        self.timestamp.pop(boxid)
        self.box_Id.pop(boxid)
        self.box_Param.pop(boxid)
        self.annotation.pop(boxid)

    #Remove high level events
    def removeEvent(self,boxid,action):
        global frameCounter
        #boxid is the index of boxes
        for key in self.annotation[boxid]:
            if action == key:
                self.annotation[boxid].remove(key)
        frameNumber = frameCounter + 1
        #Annotate the box at remaining frames
        while frameNumber < len(player.time_buff):
            if boxid >= len(player.videobox[frameNumber].box_Id):
                break
            if action in player.videobox[frameNumber].annotation[boxid]:
                player.videobox[frameNumber].annotation[boxid].remove(action)
            frameNumber += 1

    #Handles the annotation for basic and high level classes
    def changeClass(self,boxid,classify):
        global classLabels, highLabels
        if classify in classLabels:
            for annot in self.annotation[boxid]:
                if annot in classLabels or annot == 'Clear':
                    self.annotation[boxid].remove(annot)
                    self.annotation[boxid].append(classify)
        elif classify in highLabels:
            if classify in self.annotation[boxid]:
                pass
            else:
                self.annotation[boxid].append(classify)


class videoGantChart(FigureCanvas):
    def __init__(self, parent=None,width=15,height=1,dpi=100):
        gantChart = Figure(figsize=(width, height), dpi=dpi)
        self.axes = gantChart.add_subplot(111)

        self.drawChart()

        FigureCanvas.__init__(self, gantChart)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def drawChart(self):
        pass

#Class for the gantChart
class gantShow(videoGantChart):
    #Plot the chart
    def drawChart(self):
        global imageBuffer, framerate
        global annotationColors
        global xTicks
        global classLabels,gantEnabled

        temp_action = []
        self.timeWithId = []
        self.tickY = []
        self.tickX = []
        self.boxAtYaxes = []
        self.axes.hlines(0,0,0)

        time_index = 0
        #X axis with 5 sec timestep
        for index in range(len(imageBuffer)):
            if index % int(round(framerate)) == 0:
                self.tickX.append(time_index)
                time_index += 1

        if gantEnabled:
            for frame_index in player.videobox:
                for boxIdx in frame_index.box_Id:
                    if boxIdx > frame_index.box_Id[-1]:
                        break
                    for allactions in frame_index.annotation[boxIdx]:
                        #print 'action', action
                        if isinstance(allactions, list):
                            for action in allactions:
                                self.boxAtYaxes.append([boxIdx,action])
                                self.timeWithId.append([boxIdx,frame_index.timestamp[frame_index.box_Id.index(boxIdx)],action])
                        else:
                            self.boxAtYaxes.append([boxIdx,allactions])
                            self.timeWithId.append([boxIdx,frame_index.timestamp[frame_index.box_Id.index(boxIdx)],frame_index.annotation[frame_index.box_Id.index(boxIdx)]])
            #print self.timeWithId
            #Remove duplicates and sort the Y axes
            self.boxAtYaxes.sort()
            self.boxAtYaxes = list(k for k,_ in itertools.groupby(self.boxAtYaxes))
            #The line above sorts Y axes by its action
            #self.boxAtYaxes.sort(key=lambda x: x[1])


            for key in range(len(self.boxAtYaxes)):
                self.tickY.append(key)
            for index in range(len(self.timeWithId)):
                for action in self.timeWithId[index][2]:
                    self.startTime,self.endTime = self.timeCalc(self.timeWithId,index,action)
                    if self.timeWithId[index][1] == self.endTime:
                        self.color = self.getColor(action)
                        self.axes.hlines(self.boxAtYaxes.index([self.timeWithId[index][0],action]), self.startTime,self.endTime+(1/framerate),linewidth=8,color=self.color)
                    #To else ti fash??
                    else:
                        self.color = self.getColor(action)
                        #print 'Color:', self.color
                        self.axes.hlines(self.boxAtYaxes.index([self.timeWithId[index][0],action]), self.startTime,self.endTime,linewidth=8,color=self.color)

        for tick in self.axes.yaxis.get_major_ticks():
            tick.label.set_fontsize(9)

        #self.axes.set_xticks(self.tickX)
        self.axes.set_xticklabels([])
        #self.axes.get_xaxis().set_visible(False)
        self.axes.set_yticks(self.tickY)
        self.axes.set_ylim([-1,len(self.boxAtYaxes)])
        self.axes.set_yticklabels(['<'+str(index[0])+'>::'+index[1] for index in self.boxAtYaxes])
        self.axes.grid(True)

    #Calculates the end time for each annotation to plot
    def timeCalc(self,time,curr,activity):
        #temp_class = time[curr][2]
        temp_id = time[curr][0]
        startTime = time[curr][1]
        endTime = time[curr][1]
        while activity in time[curr][2] and temp_id in time[curr]:
            endTime = time[curr][1]
            curr += 1
            if curr > len(time)-1:
                break
        return startTime,endTime

    #Calculates the color for the gantChart and bound Boxes
    def getColor(self,label):
        global classLabels,highLabels
        if label == 'Clear':
                #color = 'Clear'
            return '#0000FF'
        elif label in classLabels:
                #color = label
            return annotationColors[classLabels.index(label) % len(classLabels)]
        elif label in highLabels:
            return eventColors[highLabels.index(label) % len(highLabels)]
    '''
    def color_variant(self,hex_color, brightness_offset=1):
        """ takes a color like #87c95f and produces a lighter or darker variant """
        #print 'type:::', type(hex_color)
        if len(hex_color) != 7:
            raise Exception("Passed %s into color_variant(), needs to be in #87c95f format." % hex_color)
        rgb_hex = [hex_color[x:x+2] for x in [1, 3, 5]]
        new_rgb_int = [int(hex_value, 16) + brightness_offset for hex_value in rgb_hex]
        new_rgb_int = [min([255, max([0, i])]) for i in new_rgb_int] # make sure new values are between 0 and 255
        # hex() produces "0x88", we want just "88"
        return "#" + "".join([hex(i)[2:] for i in new_rgb_int])

    #Returns the added hex
    def add_hex(self,hex1, hex2):
        color =  hex(int(hex1, 16) + int(hex2, 16))
        color = color.lstrip('0x')
        if len(color)>6:
            color = color[:6]
        elif len(color) == 5:
            color = '0' + color
        elif len(color) == 4:
            color = '00' + color
        elif len(color) == 3:
            color = '000' + color
        elif len(color) == 2:
            color = '0000' + color
        elif len(color) == 4:
            color = '00000' + color
        return color
    '''

if __name__ == '__main__':
    app = QApplication(sys.argv)
    #rosbagAudio.runMain('rosbags/2016-07-22-13-25-52.bag')
    #rosbagAudio.runMain('test.bag')
    player = VideoPlayer()
    player.resize(640,720)
    player.show()

    sys.exit(app.exec_())
