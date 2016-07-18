#!/usr/bin/env python


#############################################################################
##
## Copyright (C) 2013 Riverbank Computing Limited.
## Copyright (C) 2013 Digia Plc and/or its subsidiary(-ies).
## All rights reserved.
##
## This file is part of the examples of PyQt.
##
## $QT_BEGIN_LICENSE:BSD$
## You may use this file under the terms of the BSD license as follows:
##
## "Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are
## met:
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above copyright
##     notice, this list of conditions and the following disclaimer in
##     the documentation and/or other materials provided with the
##     distribution.
##   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
##     the names of its contributors may be used to endorse or promote
##     products derived from this software without specific prior written
##     permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
## A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
## OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
## SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
## LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
## DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
## THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
## $QT_END_LICENSE$
##
#############################################################################

import csv
import yaml
import cv2
import os
import rosbag
import argparse
import textwrap
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


from PyQt5.QtCore import *#QDir, Qt, QUrl, QByteArray, QBuffer, QIODevice,QTime,QEvent
from PyQt5.QtGui import *#QImage
from PyQt5.QtMultimedia import *#QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import *#QVideoWidget
from PyQt5.QtWidgets import * #(QApplication, QFileDialog, QHBoxLayout, QLabel,
        #QPushButton, QSizePolicy, QSlider, QStyle, QVBoxLayout, QWidget)

#Rectangle flag variables
#global start_point #= False
#global end_point #= False
start_point = False
end_point = False
paint_bool = False
out = QByteArray()
buf = QBuffer(out)

def buffer_data(csv_file, bag, input_topic, compressed):
    image_buff = []
    time_buff  = []
    box_buff   = []
    start_time = None
    bridge     = CvBridge()

    #Buffer the bounded boxes from the csv
    if csv_file is not None and os.path.exists(csv_file):
        with open(csv_file, 'r') as file_obj:
            csv_reader = csv.reader(file_obj, delimiter = '\t')
            index = [x.strip() for x in csv_reader.next()].index('Rect_x')
            for row in csv_reader:
                (x, y, width, height) = map(int, row[index:index + 4])
                box_buff.append((x, y, width, height))

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

    return image_buff, box_buff, time_buff

def get_bag_metadata(bag):
    info_dict       = yaml.load(bag._get_yaml_info())
    topics             = info_dict['topics']
    topic            = topics[1]
    duration       = info_dict['duration']
    topic_type       = topic['type']
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

    return compressed, framerate

class VideoPlayer(QWidget):

    #global start_point
    #global end_point


    def __init__(self, parent=None):
        super(VideoPlayer, self).__init__(parent)

        global pix

        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        #add duration
        self.duration = 0
        #self.setAttribute(Qt.WA_TranslucentBackground)
        videoWidget = QVideoWidget()
        #videoWidget = QMediaObject()
        #view = QGraphicsView(self)
        scene = QGraphicsScene(self)
        #scene.setAttribute(Qt.WA_TranslucentBackground)


        #item = QGraphicsVideoItem()
        #view.setAttribute(Qt.WA_TranslucentBackground)
        #view.setSetscene(scene)
        #~ pen = QPen(Qt.red)
        #~ scene.addText("Hello, world!")

        #self.probe = QVideoProbe()
        #self.probe.videoFrameProbed.connect(self.processFrame  )
        #self.probe.setSource(self.mediaPlayer)
        #self.videoFrameProbed(self.player)

        '''
        #~ #Add this line
        view = QGraphicsView(self)
        view.setAttribute(Qt.WA_TranslucentBackground)
        view.show()
        '''

        openButton = QPushButton("Open...")
        openButton.clicked.connect(self.openFile)

        self.playButton = QPushButton()
        self.playButton.setEnabled(False)
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.play)


        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setRange(0, 0)
        self.positionSlider.sliderMoved.connect(self.setPosition)

        #Add duration Label
        self.labelDuration = QLabel()
        self.errorLabel = QLabel()
        self.errorLabel.setSizePolicy(QSizePolicy.Preferred,
                QSizePolicy.Maximum)

        controlLayout = QHBoxLayout()
        controlLayout.setContentsMargins(0, 0, 0, 0)
        controlLayout.addWidget(openButton)
        controlLayout.addWidget(self.playButton)
        controlLayout.addWidget(self.positionSlider)
        controlLayout.addWidget(self.labelDuration)

        layout = QVBoxLayout()
        layout.addWidget(videoWidget)
        #layout.addWidget(scene)
        layout.addLayout(controlLayout)
        layout.addWidget(self.errorLabel)

        self.setLayout(layout)

        #self.mediaPlayer.setVideoOutput(self.scene)
        self.mediaPlayer.setVideoOutput(videoWidget)
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)
        self.mediaPlayer.error.connect(self.handleError)

        #Test ideas
        '''
        #~ #Add this line
        view = QGraphicsView(self)
        view.setAttribute(Qt.WA_TranslucentBackground)
        view.show()
        '''

    def openFile(self):
        '''
        fileName, _ = QFileDialog.getOpenFileName(self, "Open Movie",
                "/home/dimitris/GitProjects/rosbag_annotator/2016-02-12-13-43-37.csv")
        '''
        fileName, _ = QFileDialog.getOpenFileName(self, "Open Movie", QDir.currentPath())
        print fileName

        bag = rosbag.Bag(fileName)

        #Get bag metadata
        (compressed, framerate) = get_bag_metadata(bag)

        #Buffer the rosbag, boxes, timestamps
        (image_buff, box_buff, time_buff) = buffer_data("/home/dimitris/GitProjects/rosbag_annotator/2016-02-12-13-43-37.csv", bag, "/camera/rgb/image_raw", compressed)

        fourcc = cv2.VideoWriter_fourcc('X', 'V' ,'I', 'D')
        height, width, bytesPerComponent = image_buff[0].shape
        video_writer = cv2.VideoWriter("myvid.avi", fourcc, framerate, (width,height), cv2.IMREAD_COLOR)
        if not video_writer.isOpened():
            raise ValueError("Video writer could not initialize, probably wrong file extension or path given")
        else:
            print("Video initialized")
        for frame in image_buff:
            video_writer.write(frame)
        video_writer.release()
        #~ QDataStream stream(out, QIODevice::WriteOnly);
        #~ buf.open(QIODevice.ReadWrite)
        #~ for img in image_buff:
            #~ height, width, bytesPerComponent = img.shape
            #~ bytesPerLine = 3 * width
            #~ cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            #~ QImg = QImage(img.data, width, height, bytesPerLine,QImage.Format_RGB888)
            #~ QImg.save(buf)

        #~ buf.close()

        print buf.size()

        if fileName != '':
            self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile("/home/dimitris/GitProjects/rosbag_annotator/myvid.avi")))
            #self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(QFileInfo(fileUrl).absoluteFilePath())))
            #print QFileInfo(fileUrl).absoluteFilePath()
            self.playButton.setEnabled(True)

    def play(self):
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.mediaPlayer.pause()
        else:
            self.mediaPlayer.play()

    def mediaStateChanged(self, state):
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.playButton.setIcon(
                    self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(
                    self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        #position /= 1000
        self.positionSlider.setValue(position)
        self.updateDurationInfo(position) #Add duration info

    def durationChanged(self, duration):
        #duration /= 1000
        #self.duration = duration
        self.positionSlider.setRange(0, duration)

    def setPosition(self, position):
        self.mediaPlayer.setPosition(position)

    #update duration
    def updateDurationInfo(self, currentInfo):
        duration = self.duration
        if currentInfo or duration:
            currentTime = QTime((currentInfo/3600)%60, (currentInfo/60)%60,
                    currentInfo%60, (currentInfo*1000)%1000)
            totalTime = QTime((duration/3600)%60, (duration/60)%60,
                    duration%60, (duration*1000)%1000);

            format = 'hh:mm:ss' if duration > 3600 else 'mm:ss'
            tStr = currentTime.toString(format) + " / " + totalTime.toString(format)
        else:
            tStr = ""
        self.labelDuration.setText(tStr)

    def handleError(self):
        self.playButton.setEnabled(False)
        self.errorLabel.setText("Error: " + self.mediaPlayer.errorString())

    #Mouse callback handling for Boxes
    def mousePressEvent(self,event):
        global start_point
        global end_point
        global pix
        #painter = QPainter()
        if QMouseEvent.button(event) == Qt.LeftButton:#QEvent.MouseButtonPress:
            #print 'edw'
            if start_point is True and end_point is True:
                #start_point = False
                #end_point = False
                #QPainter.eraseRect(rect)
                pass
                #QPointF.pos1 =QMouseEvent.pos()
                #draw_rect()
            elif start_point is False:
                #QPointF.pos1 =QMouseEvent.pos()
                QPoint.pos1 = QMouseEvent.pos(event)
                start_point = True
                print start_point
            elif end_point is False:
                QPoint.pos2 = QMouseEvent.pos(event) # QEvent.MouseButtonPress.pos()
                end_point = True
                #print end_point
                rect = QRect(QPoint.pos1,QPoint.pos2)
                p_event = QPaintEvent(rect)
                #player.paintEvent(rect)
                #painter.drawRect(rect)
                #QPaintEvent(rect)
                self.update(rect)

    def paintEvent(self,event):
        global start_point
        global end_point

        #global videoGraph
        if start_point is True and end_point is True:
            start_point = False
            end_point = False
            painter = QPainter(self)
            painter.begin(self)
            painter.setBrush(QColor(200, 0, 0))
            print event.rect()
            painter.drawRect(event.rect())
            painter.end()
        #~ QPaintEvent(event)



'''
class Graphics(VideoPlayer):
    def __init__(self,parent=VideoPlayer):
        super(Graphics,self).__init__(parent=VideoPlayer)

        #self.view = QGraphicsView(self)
        #self.view.setAttribute(Qt.WA_TranslucentBackground)
        #view.show()
'''

'''
    def paintEvent(event):
        painter = QPainter(self)
        pass

    def draw_rect(self,pos1,pos2):
        QRect(pos1,pos2)
        pass
'''

if __name__ == '__main__':

    import sys

    app = QApplication(sys.argv)

    player = VideoPlayer()
    #drawer = Graphics()
    #drawer.resize(640,480)
    player.resize(640, 480)
    player.show()
    #drawer.show()


    sys.exit(app.exec_())
