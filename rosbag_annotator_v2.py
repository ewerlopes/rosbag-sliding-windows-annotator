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
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *

start_point = False
end_point = False
boxInitialized = False

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
    if csv_file is not None and os.path.exists(csv_file):
        with open(csv_file, 'r') as file_obj:
            csv_reader = csv.reader(file_obj, delimiter = '\t')
            try:
                index = [x.strip() for x in csv_reader.next()].index('Rect_id')
                print index
                for row in csv_reader:
                    (rec_id,x, y, width, height) = map(int, row[index:index + 5])
                    box_buff.append((rec_id,x, y, width, height))
            except:
                print "Malakiaa egine"
                return False#,False
            return box_buff
    else:
        print "file error"
        return False#,False


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

    return message_count,duration,compressed, framerate


class VideoWidgetSurface(QAbstractVideoSurface):

    def __init__(self, widget, parent=None):
        super(VideoWidgetSurface, self).__init__(parent)
        self.widget = widget
        self.imageFormat = QImage.Format_Invalid
        #self.video = VideoPlayer #malakies

    def supportedPixelFormats(self, handleType=QAbstractVideoBuffer.NoHandle):
        formats = [QVideoFrame.PixelFormat()]
        if (handleType == QAbstractVideoBuffer.NoHandle):
            for f in [QVideoFrame.Format_RGB32, QVideoFrame.Format_ARGB32, QVideoFrame.Format_ARGB32_Premultiplied, QVideoFrame.Format_RGB565, QVideoFrame.Format_RGB555,QVideoFrame.Format_BGR24,QVideoFrame.Format_RGB24]:
                formats.append(f)
        return formats

    def isFormatSupported(self, _format):
        print 'formatSupport'
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
        self.annotColor = Qt.blue
        self.deleteEnabled = False

    def videoSurface(self):
        return self.surface

    #Shows the right click menu
    def contextMenuEvent(self,event):
        if event.reason() == QContextMenuEvent.Mouse:
            self.annotEnabled = True
            menu = QMenu(self)
            clear = menu.addAction('Clear')
            for i in player.json_Labels:
                menu.addAction(i)
            deleteBox = menu.addAction('Delete Box');
            action = menu.exec_(self.mapToGlobal(event.pos()))
            if action == deleteBox:
                self.deleteEnabled = True
            '''
            if action == sit:
                self.annotClass = 'Sit' #Qt.magenta
                self.annotColor = Qt.magenta
            elif action == walk:
                self.annotClass = 'Walk' #Qt.yellow
                self.annotColor = Qt.yellow
            elif action == stand:
                self.annotClass = 'Stand' #Qt.cyan
                self.annotColor = Qt.cyan
            elif action == clear:
                self.annotClass = 'Clear' #Qt.blue
                self.annotColor = Qt.blue
            '''
            self.posX_annot = event.pos().x()
            self.posY_annot = event.pos().y()
            self.repaint()

        self.deleteEnabled = False
        self.annotEnabled = False

    def sizeHint(self):
        return self.surface.surfaceFormat().sizeHint()

    def paintEvent(self, event):
        global start_point
        global end_point
        global frameCounter
        global removeBool
        global timeId

        painter = QPainter(self)
        rectPainter = QPainter(self)

        if not rectPainter.isActive() :
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
            for i in range(len(player.videobox[frameCounter].box_Id)):
                x,y,w,h = player.videobox[frameCounter].box_Param[i]
                if self.posX_annot > x and self.posX_annot < (x+w) and self.posY_annot > y and self.posY_annot < (y+h):
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(Qt.red)
                    rectPainter.drawRect(x,y,w,h)
                    player.videobox[frameCounter].removeSpecBox(i)

        #Enabled when annotating
        elif self.annotEnabled:
            for i in range(len(player.videobox[frameCounter].box_Id)):
                x,y,w,h = player.videobox[frameCounter].box_Param[i]
                if self.posX_annot > x and self.posX_annot < (x+w) and self.posY_annot > y and self.posY_annot < (y+h):
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(self.annotColor)
                    rectPainter.drawRect(x,y,w,h)
                    player.videobox[frameCounter].changeClass(i,self.annotClass)
                    box = i
                    self.frameNumber = frameCounter+1 #Begin annotate from the next frame
                    break
            while self.frameNumber < len(player.time_buff):
                if box >= len(player.videobox[frameCounter].box_Id):
                    break
                player.videobox[self.frameNumber].changeClass(box,self.annotClass)
                self.frameNumber += 1

        elif start_point is True and end_point is True:
            x = event.rect().x()
            y = event.rect().y()
            w = event.rect().width()
            h = event.rect().height()

            rectPainter.setRenderHint(QPainter.Antialiasing)
            rectPainter.setPen(Qt.blue)
            rectPainter.drawRect(x,y,w,h)
            #Remove old boxes and add the new ones
            if self.enableWriteBox:
                if removeBool:
                    try:
                        timeId = player.videobox[frameCounter].timestamp[0] #Keep the timestamp to add the new box
                    except:
                        pass
                    player.videobox[frameCounter].removeAllBox() #Deletes all boxes in current frame

                boxNumber = len(player.videobox[frameCounter].box_Id)
                print player.videobox[frameCounter].box_Id,player.videobox[frameCounter].timestamp
                player.videobox[frameCounter].addBox(timeId,[boxNumber,x,y,w,h],'Clear')

        #Play the bound boxes from csv
        elif len(player.videobox) > 0 and frameCounter < len(player.time_buff) and not  self.vanishBox:
                for i in range(len(player.videobox[frameCounter].box_Id)):
                    x,y,w,h = player.videobox[frameCounter].box_Param[i]
                    rectPainter.setPen(self.annotColor)
                    rectPainter.drawRect(x,y,w,h)
                    #player.videobox[frameCounter].removeBox()

        self.vanishBox = False
        if rectPainter.isActive():
            rectPainter.end()

    #Mouse callback handling for Boxes
    def mousePressEvent(self,event):
        global start_point
        global end_point,removeBool

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
                self.vanishBox = True
                self.repaint()
                self.enableWriteBox = True
                self.repaint(rect)
                removeBool = False
                self.enableWriteBox = False
                start_point = False
                end_point = False

    def resizeEvent(self, event):
        QWidget.resizeEvent(self, event)
        self.surface.updateVideoRect()

class VideoPlayer(QWidget):
    def __init__(self, parent=None):
        super(VideoPlayer, self).__init__(parent)
        self.videobox = []
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

        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setRange(0, 0)
        self.positionSlider.sliderMoved.connect(self.setPosition)

        self.controlLayout = QHBoxLayout()
        self.controlLayout.setContentsMargins(0, 0, 0, 0)
        self.controlLayout.addWidget(self.openButton)
        self.controlLayout.addWidget(self.importCsv)
        self.controlLayout.addWidget(self.playButton)
        self.controlLayout.addWidget(self.positionSlider)
        self.controlEnabled = False

        layout = QVBoxLayout()
        layout.addWidget(self.videoWidget)
        layout.addLayout(self.controlLayout)

        self.setLayout(layout)

        self.mediaPlayer.setVideoOutput(self.videoWidget.videoSurface())
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)

    def openFile(self):
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
            (self.image_buff, self.time_buff) = buffer_data(bag, "/camera/rgb/image_raw", compressed)
            fourcc = cv2.VideoWriter_fourcc('X', 'V' ,'I', 'D')
            height, width, bytesPerComponent = self.image_buff[0].shape
            video_writer = cv2.VideoWriter("myvid.avi", fourcc, framerate, (width,height), cv2.IMREAD_COLOR)

            if not video_writer.isOpened():
                self.errorMessages(2)
            else:
                print("Video initialized")
                for frame in self.image_buff:
                    video_writer.write(frame)
                video_writer.release()

                self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile("/home/dimitris/GitProjects/rosbag_annotator/myvid.avi")))
                #self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(QFileInfo(fileUrl).absoluteFilePath())))
                #print QFileInfo(fileUrl).absoluteFilePath()
                self.playButton.setEnabled(True)

    #Open CSV file
    def openCsv(self):
        self.json_Labels = []
        fileName,_ =  QFileDialog.getOpenFileName(self, "Open Csv ", QDir.currentPath(),"*.csv")
        #print type(fileName)
        box_buff = buffer_csv(fileName)

        if not box_buff :
            print type(box_buff)
            self.errorMessages(1)
        else:
            self.box_buffer = [list(elem) for elem in box_buff]
            #Initialize objects which are equal to frames
            self.videobox = [boundBox(count) for count in range(len(self.time_buff))]
            #Frame counter initialize
            counter = -1
            for idx,key in enumerate(self.box_buffer):
                if key[0] == 0:
                    counter += 1
                    self.videobox[counter].addBox(self.time_buff[counter],key,'Clear')
                else:
                    self.videobox[counter].addBox(self.time_buff[counter],key,'Clear')
            #Parse json file
            try:
                self.json_Labels = self.parseJson()
            except:
                self.errorMessages(3)

    def parseJson(self):
        with open("labels.json") as json_file:
                json_data = json.load(json_file)
                json_label = []
                print json_data
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
        list_insert_param_1 = []
        list_insert_param_2 = []
        list_insert_param_3 = []
        list_insert_param_4 = []
        list_insert_class = []

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

        with open('boxes_updated.csv', 'w') as file:
            csv_writer = csv.writer(file, delimiter='\t')
            headlines = ['Timestamp','Rect_id', 'Rect_x','Rect_y','Rect_W','Rect_H','Class','Meter_X','Meter_Y','Meter_Z','Top','Height' ,'Distance']
            csv_writer.writerow(headlines)
            rows = zip(list_insert_time,list_insert_box,list_insert_param_1,list_insert_param_2,list_insert_param_3,list_insert_param_4,list_insert_class)
            csv_writer.writerows(rows)

    def closeEvent(self,event):
        self.writeCSV(self.videobox)

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

    def removeAllBox(self):#,frameCounter):
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
        if boxid  < len(self.annotation):
            self.annotation.pop(boxid)
        self.annotation.insert(boxid,classify)

if __name__ == '__main__':
    app = QApplication(sys.argv)

    player = VideoPlayer()
    player.resize(480,320)
    player.show()

    sys.exit(app.exec_())

