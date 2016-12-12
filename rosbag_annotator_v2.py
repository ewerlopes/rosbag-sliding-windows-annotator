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
import numpy as np
from matplotlib.widgets import Cursor
from numpy import arange, sin, pi
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.transforms as transforms
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

start_point = False
end_point = False
boxInitialized = False
annotationColors = ['#00FF00', '#FF00FF','#FFFF00','#00FFFF','#FFA500']
gantEnabled = False
frameCounter = 0

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
            cv_image = cv2.imdecode(nparr, 1) #### cv2.CV_LOAD_IMAGE_COLOR has as enum value 1.
            ## TODO: fix the problem with the this enum value.

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
                    #index = [x.strip() for x in row_1].index('Rect_id')
                    for row in csv_reader:
                        (rec_id,x, y, width, height) = map(int, row[index:index + 5])
                        #action = map(str,row[index+5])
                        (meter_X,meter_Y,meter_Z,top,meter_h,distance) = map(float, row[(index+6)::])
                        box_buff.append((rec_id,x, y, width, height))
                        box_buff_action.append(str(row[index+5]))
                        metrics.append((meter_X,meter_Y,meter_Z,top,meter_h,distance))
            except:
                return False,False,False
            return box_buff,metrics,box_buff_action
    else:
        return False,False,False

def get_bag_metadata(bag):
    info_dict       = yaml.load(bag._get_yaml_info())
    topics             = info_dict['topics']
    topic            = topics[0]
    duration       = info_dict['duration']
    topic_type       = topic['type']
    message_count = topic['messages']

    #Messages for test
    print "\nRosbag topics found: "
    for top in topics:
        print "\t- ", top["topic"], "\n\t\t-Type: ", top["type"],"\n\t\t-Fps: ", top["frequency"]

    print 23 * '#'
    print topic_type
    #Checking if the topic is compressed
    if 'CompressedImage' in topic_type:
        compressed = True
    else:
        compressed = False

    #Get framerate
    framerate = message_count/duration

    return message_count,duration,compressed, framerate


class VideoWidgetSurface(QAbstractVideoSurface):

    def __init__(self, widget, parent=None):
        super(VideoWidgetSurface, self).__init__(parent)
        self.widget = widget
        self.imageFormat = QImage.Format_Invalid

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
        global frameCounter
        imageFormat = QVideoFrame.imageFormatFromPixelFormat(_format.pixelFormat())
        size = _format.frameSize()
        frameCounter = 0 #Frame Counter initialize
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
        classLabels = []
        imageBuffer = []


    def videoSurface(self):
        return self.surface

    #Shows the right click menu
    def contextMenuEvent(self,event):
        global posX
        global posY
        global classLabels,gantChart,gantEnabled

        if event.reason() == QContextMenuEvent.Mouse:
            menu = QMenu(self)
            clear = menu.addAction('Clear')

            for i in classLabels:
                self.buttonLabels.append(menu.addAction(i))

            deleteBox = menu.addAction('Delete Box')
            deleteAllBoxes = menu.addAction('Delete All Boxes')
            changeId = menu.addAction('Change Id')
            cancel = menu.addAction('Cancel')
            action = menu.exec_(self.mapToGlobal(event.pos()))
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

            self.posX_annot = event.pos().x()
            self.posY_annot = event.pos().y()

            posX = event.pos().x()
            posY = event.pos().y()

            self.repaint()
            self.buttonLabels = []
        self.annotEnabled = False

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
                if self.posX_annot > x and self.posX_annot < (x+w) and self.posY_annot > y and self.posY_annot < (y+h):
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
                if self.posX_annot > x and self.posX_annot < (x+w) and self.posY_annot > y and self.posY_annot < (y+h):
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(QColor(self.getColorBox(self.annotClass)))
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()
                    player.videobox[frameCounter].changeClass(i,self.annotClass)
                    box = i
                else:
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

            #Annotate the box at remaining frames
            while self.frameNumber < len(player.time_buff):
                if box >= len(player.videobox[self.frameNumber].box_Id) or box is None:
                    break
                player.videobox[self.frameNumber].changeClass(box,self.annotClass)
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
                    player.videobox[frameCounter].addBox(timeId,[boxNumber,x,y,w,h],'Clear')
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
        global classLabels
        for index,key in enumerate(classLabels):
            if action == key:
                return annotationColors[index % len(annotationColors)]
            elif action == 'Clear':
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

class VideoPlayer(QWidget):
    def __init__(self, parent=None):
        global gantChart
        super(VideoPlayer, self).__init__(parent)
        self.videobox = []
        self.metric_buffer = []

        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)

        self.videoWidget = VideoWidget()
        self.openButton = QPushButton("Open...")
        self.importCsv = QPushButton("Import CSV...")
        self.openButton.clicked.connect(self.openFile)
        self.importCsv.clicked.connect(self.openCsv)

        self.playButton = QPushButton()
        self.playButton.setEnabled(False)
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.play)

        self.nexstDWindowButton = QPushButton()
        self.nexstDWindowButton.setEnabled(False)
        self.nexstDWindowButton.setIcon(self.style().standardIcon(QStyle.SP_MediaSkipForward))

        self.previousDWindowButton = QPushButton()
        self.previousDWindowButton.setEnabled(False)
        self.previousDWindowButton.setIcon(self.style().standardIcon(QStyle.SP_MediaSkipBackward))

        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setRange(0, 0)
        self.positionSlider.sliderMoved.connect(self.setPosition)

        # Create a label widget with our text
        self.label = QLabel('Hello, world!')

        self.controlLayout = QHBoxLayout()
        self.controlLayout.setContentsMargins(0, 0, 0, 0)
        self.controlLayout.addWidget(self.openButton)
        self.controlLayout.addWidget(self.importCsv)
        self.controlLayout.addWidget(self.previousDWindowButton)
        self.controlLayout.addWidget(self.playButton)
        self.controlLayout.addWidget(self.nexstDWindowButton)
        self.controlLayout.addWidget(self.positionSlider)
        self.controlEnabled = False

        self.annotationLayout = QHBoxLayout()
        self.annotationLayout.setContentsMargins(0, 0, 0, 0)


        self.act_group_box = QGroupBox(title="Activity")
        self.exp_group_box = QGroupBox(title="Expectation")
        self.ctrl_group_box = QGroupBox(title="Control")
        self.act_group_box.setFlat(True)
        self.exp_group_box.setFlat(True)
        self.ctrl_group_box.setFlat(True)

        styleSheet = " \
                QGroupBox {\
                    border: 1px solid gray;\
                    border-radius: 9px;\
                    margin-top: 0.5em;\
                }\
                QGroupBox::title {\
                    subcontrol-origin: margin;\
                    left: 10px;\
                    padding: 0 3px 0 3px;\
                }"

        self.act_group_box.setStyleSheet(styleSheet)


        # Create an array of radio buttons for the given tag option
        self.act_opts = [QRadioButton("Very low"), QRadioButton("Low"), QRadioButton("Medium"),
                         QRadioButton("High"), QRadioButton("Very High")]
        self.exp_opts = [QRadioButton("Very low"), QRadioButton("Low"), QRadioButton("Medium"),
                         QRadioButton("High"), QRadioButton("Very High")]
        self.ctrl_opts = [QRadioButton("Very low"), QRadioButton("Low"), QRadioButton("Medium"),
                         QRadioButton("High"), QRadioButton("Very High")]

        # Set a radio button to be checked by default
        self.act_opts[0].setChecked(True)

        # Radio buttons usually are in a vertical layout
        self.act_button_layout = QVBoxLayout()

        # Create a button group for radio buttons
        self.act_button_group = QButtonGroup()

        for i in range(len(self.act_opts)):
            # Add each radio button to the button layout
            self.act_button_layout.addWidget(self.act_opts[i])
            # Add each radio button to the button group & give it an ID of i
            self.act_button_group.addButton(self.act_opts[i], i)
            # Connect each radio button to a method to run when it's clicked
            #self.connect(act_opts[i], SIGNAL("clicked()"), self.radio_button_clicked)

        # Set the layout of the group box to the button layout
        self.act_button_layout.addStretch(1)
        self.act_group_box.setLayout(self.act_button_layout)
        self.annotationLayout.addWidget(self.act_group_box)

        self.gantt = gantShow()
        gantChart = self.gantt

        layout = QVBoxLayout()
        layout.addWidget(self.videoWidget)
        layout.addLayout(self.controlLayout)
        layout.addLayout(self.annotationLayout)
        layout.addWidget(self.gantt)

        self.setLayout(layout)

        self.mediaPlayer.setVideoOutput(self.videoWidget.videoSurface())
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)

    def openFile(self):
        global imageBuffer,framerate
        fileName,_ = QFileDialog.getOpenFileName(self, "Open Bag", QDir.currentPath(),"*.bag")

        if not fileName:
            pass
        else:
            try:
                bag = rosbag.Bag(fileName)
            except:
                self.errorMessages(0)

            #Get bag metadata
            (self.message_count,self.duration,compressed, framerate) = get_bag_metadata(bag)
            #Buffer the rosbag, boxes, timestamps
            (imageBuffer, self.time_buff) = buffer_data(bag, "/ext_usb_camera/image/compressed", compressed)
            fourcc = cv2.VideoWriter_fourcc('X', 'V' ,'I', 'D')
            height, width, bytesPerComponent = imageBuffer[0].shape
            video_writer = cv2.VideoWriter("myvid.avi", fourcc, framerate, (width,height), cv2.IMREAD_COLOR)

            if not video_writer.isOpened():
                self.errorMessages(2)
            else:
                print("Video initialized")
                for frame in imageBuffer:
                    video_writer.write(frame)
                video_writer.release()

                self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile("/home/ewerlopes/developer/rosbag_annotator/myvid.avi")))
                self.playButton.setEnabled(True)

    #Open CSV file
    def openCsv(self):
        global classLabels,gantEnabled
        self.box_buffer =[]

        fileName,_ =  QFileDialog.getOpenFileName(self, "Open Csv ", QDir.currentPath(),"*.csv")
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
                        self.videobox[counter].addBox(self.time_buff[counter],key,'Clear')
                    else:
                        self.videobox[counter].addBox(self.time_buff[counter],key,'Clear')

            #Parse json file
            try:
                classLabels = self.parseJson()
            except:
                self.errorMessages(3)

            gantEnabled = True
            gantChart.axes.clear()
            gantChart.drawChart()
            gantChart.draw()

    def parseJson(self):
        with open("labels.json") as json_file:
                json_data = json.load(json_file)
                json_label = []
                for i in json_data['labels'] :
                    json_label.append(i)
        return json_label

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
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.mediaPlayer.pause()
        else:
            self.mediaPlayer.play()

    def mediaStateChanged(self, state):
        if state == QMediaPlayer.PlayingState:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        self.positionSlider.setValue(position)

    def keyPressEvent(self,event):
        if event.key() == Qt.Key_Control:
            self.controlEnabled = True

    def keyReleaseEvent(self,event):
        if event.key() == Qt.Key_Control:
            self.controlEnabled = False

    def durationChanged(self, duration):
        self.positionSlider.setRange(0, duration)

    #Allazei otan peiraksw ton slider!!
    def setPosition(self, position):
        global frameCounter
        frameCounter = int(round(self.message_count * position/(self.duration * 1000)))
        self.mediaPlayer.setPosition(position)

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

    def changeClass(self,boxid,classify):
        if boxid < len(self.annotation):
            self.annotation.pop(boxid)
        self.annotation.insert(boxid,classify)

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
            for box_index in player.videobox:
                for boxIdx in box_index.box_Id:
                    if boxIdx > len(box_index.box_Id):
                        break
                    self.boxAtYaxes.append([boxIdx,box_index.annotation[box_index.box_Id.index(boxIdx)]])
                    self.timeWithId.append([boxIdx,box_index.timestamp[box_index.box_Id.index(boxIdx)],box_index.annotation[box_index.box_Id.index(boxIdx)]])

            #Remove duplicates for the y axis
            temp_set = set(map(tuple,self.boxAtYaxes))
            self.boxAtYaxes = sorted(map(list,temp_set))

            for key in range(len(self.boxAtYaxes)):
                self.tickY.append(key)
            for index in range(len(self.timeWithId)):
                self.startTime,self.endTime = self.timeCalc(self.timeWithId,index)
                if self.timeWithId[index][1] == self.endTime:
                    self.color = self.getColor(self.timeWithId[index][2])
                    self.axes.hlines(self.boxAtYaxes.index([self.timeWithId[index][0],self.timeWithId[index][2]]), self.startTime,self.endTime+(1/framerate),linewidth=8,color=self.color)
                self.color = self.getColor(self.timeWithId[index][2])
                self.axes.hlines(self.boxAtYaxes.index([self.timeWithId[index][0],self.timeWithId[index][2]]), self.startTime,self.endTime,linewidth=8,color=self.color)

        for tick in self.axes.yaxis.get_major_ticks():
            tick.label.set_fontsize(9)

        self.axes.set_xticks(self.tickX)
        self.axes.set_yticks(self.tickY)
        self.axes.set_ylim([-1,len(self.boxAtYaxes)])
        self.axes.set_yticklabels(['<'+str(index[0])+'>::'+index[1] for index in self.boxAtYaxes])
        self.axes.grid(True)

    #Calculates the end time for each annotation to plot
    def timeCalc(self,time,curr):
        temp_class = time[curr][2]
        temp_id = time[curr][0]
        starttime = time[curr][1]
        endtime = time[curr][1]
        while temp_class in time[curr] and temp_id in time[curr]:
            endtime = time[curr][1]
            curr += 1
            if curr > len(time)-1:
                break
        return starttime,endtime

    #Calculates the color for the gantChart and bound Boxes
    def getColor(self,action):
        global classLabels
        for index,key in enumerate(classLabels):
            if action == key:
                return annotationColors[index % len(annotationColors)]
            elif action == 'Clear':
                return '#0000FF'

if __name__ == '__main__':
    app = QApplication(sys.argv)

    player = VideoPlayer()
    player.resize(640,920)
    player.show()

    sys.exit(app.exec_())

