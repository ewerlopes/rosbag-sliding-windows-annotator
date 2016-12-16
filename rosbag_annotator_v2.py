#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""The Video Widget example shows how to implement a video widget
using QtMultimedia's QAbstractVideoSurface.

The following is a translation into PyQt5 from the C++ example found in
C:\QtEnterprise\5.1.1\msvc2010\examples\multimediawidgets\customvideosurface\customvideowidget."""
from __future__ import division
import csv
import cv2
import os
import rosbag
import logging
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
import sys, copy
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *
import warnings
from matplotlib.widgets import Cursor
from numpy import arange, sin, pi
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.transforms as transforms
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from annotator_utils import *

start_point = False
end_point = False
boxInitialized = False
annotationColors = ['#00FF00', '#FF00FF','#FFFF00','#00FFFF','#FFA500']
gantEnabled = False

### logging setup #####
logger = logging.getLogger(__name__)
handler = QtHandler()
format = '%(asctime)s -- %(levelname)s --> %(message)s'
date_format = '%Y-%m-%d %H:%M:%S'
handler.setFormatter(logging.Formatter(format,date_format))
logger.addHandler(handler)
logger.setLevel(logging.DEBUG)

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

    def sizeHint(self):
        return self.surface.surfaceFormat().sizeHint()

    #Shows the video
    def paintEvent(self, event):
        global start_point
        global end_point
        global frameCounter
        global timeId

        painter = QPainter(self)
        rectPainter = QPainter(self)

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

        if rectPainter.isActive():
            rectPainter.end()

    def resizeEvent(self, event):
        QWidget.resizeEvent(self, event)
        self.surface.updateVideoRect()

class VideoPlayer(QWidget):
    def __init__(self, parent=None):
        global gantChart
        super(VideoPlayer, self).__init__(parent)
        self.videobox = []
        self.metric_buffer = []
        self.filename = ''
        self.isUnsave = True
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)

        # the jason config data for setting labels
        self.label_configs = self.parseConfig()
        self.data = {}
        self.types = {}          # This loads the type of objects in the treeviewer. Used for saying which topic to save.
        self.isBagLoaded = False
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
        self.nexstDWindowButton.clicked.connect(self.moveWindowForward)

        self.previousDWindowButton = QPushButton()
        self.previousDWindowButton.setEnabled(False)
        self.previousDWindowButton.setIcon(self.style().standardIcon(QStyle.SP_MediaSkipBackward))
        self.previousDWindowButton.clicked.connect(self.moveWindowBackward)

        self.reloadButton = QPushButton("Load")
        self.reloadButton.setEnabled(False)
        self.reloadButton.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        self.reloadButton.clicked.connect(self.reload)

        self.saveButton = QPushButton("Save")
        self.saveButton.setEnabled(False)
        self.saveButton.clicked.connect(self.save)

        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setRange(0, 0)
        self.positionSlider.sliderPressed.connect(self.setPosition)


        self.tree_of_topics = QTreeWidget()
        self.tree_of_topics.setHeaderLabel("Save topics:")
        self.tree_of_topics.setMaximumWidth(500)

        ### BUTTONS FOR THE SECOND CONTROL BUTTON LAYOUT
        # Create a label widget for buttons in the second layout
        self.winsize_spinbox_label = QLabel('Window size: ')
        self.overlap_combo_label = QLabel('Overlap:')
        self.topics_combo_label = QLabel('Image Topics:')
        self.windows_combo_label = QLabel('Window:')
        self.taggeds_label = QLabel("Annotated windows: ")
        self.logOutput_label = QLabel("Log area:")
        self.duration_label = QLabel("--:--")

        #Creat spinner box (windows size)
        self.windowSize_spinBox = QSpinBox()
        self.windowSize_spinBox.valueChanged.connect(self.windowSizeChanged)
        self.windowSize_spinBox.setValue(3)

        #Create overlap dropdown list
        self.overlap_combo_box = QComboBox()
        for s in range(0,101,5):
            self.overlap_combo_box.addItem(str(s) + '%')
        self.overlap_combo_box.currentIndexChanged.connect(self.overlapComboxChanged)
        self.overlap_combo_box.setCurrentIndex(10)

        # Create overlap dropdown list
        self.topics_combo_box = QComboBox()
        self.topics_combo_box.currentIndexChanged.connect(self.topicsComboxChanged)
        self.windows_combo_box = QComboBox()
        self.windows_combo_box.currentIndexChanged.connect(self.windowsComboxChanged)

        # Create log area
        self.logWindowsTagged = QTextEdit()
        self.logWindowsTagged.setReadOnly(True)
        self.logWindowsTagged.setLineWrapMode(QTextEdit.WidgetWidth)
        self.logWindowsTagged.setMaximumHeight(50)
        self.logWindowsTagged.setTextColor(QColor("green"))

        # Create log area
        self.logOutput = QTextEdit()
        self.logOutput.setReadOnly(True)
        self.logOutput.setLineWrapMode(QTextEdit.WidgetWidth)

        self.log_font = self.logOutput.font()
        self.log_font.setFamily("Courier")
        self.log_font.setPointSize(10)

        self.logOutput.moveCursor(QTextCursor.End)
        self.logOutput.setCurrentFont(self.log_font)
        self.logOutput.setTextColor(QColor("red"))

        self.scroll_bar = self.logOutput.verticalScrollBar()
        self.scroll_bar.setValue(self.scroll_bar.maximum())

        XStream.stdout().messageWritten.connect(self.logOutput.append)
        XStream.stderr().messageWritten.connect(self.logOutput.append)

        ## USE THIS TO SAVE THE OUTPUT
        # with open('log.txt', 'w') as yourFile:
        #    yourFile.write(str(yourQTextEdit.toPlainText()))

        self.topics_to_save = {}

        # Create tabs
        self.tab_container = QTabWidget()
        self.tabs = dict([])
        self.annotation_layouts = dict([])
        for t_name in self.label_configs.keys():
            self.tabs[t_name] = QWidget()
            self.annotation_layouts[t_name] = QHBoxLayout()
            self.annotation_layouts[t_name].setContentsMargins(0, 0, 0, 0)

        #parse labels
        self.tabs_labels = dict([])
        for t_name in self.label_configs.keys():
            self.tabs_labels[t_name] = self.label_configs[t_name]

        self.control_button_layout1 = QHBoxLayout()
        self.control_button_layout2 = QHBoxLayout()
        self.control_button_layout3 = QHBoxLayout()
        self.control_button_layout4 = QVBoxLayout()

        self.control_button_layout1.setContentsMargins(0, 0, 0, 0)
        self.control_button_layout1.addWidget(self.openButton)
        self.control_button_layout1.addWidget(self.saveButton)
        #self.control_button_layout1.addWidget(self.importCsv)
        self.control_button_layout1.addWidget(self.reloadButton)
        self.control_button_layout1.addWidget(self.previousDWindowButton)
        self.control_button_layout1.addWidget(self.playButton)
        self.control_button_layout1.addWidget(self.nexstDWindowButton)
        self.control_button_layout1.addWidget(self.windows_combo_label)
        self.control_button_layout1.addWidget(self.windows_combo_box)
        self.control_button_layout1.addWidget(self.positionSlider)
        self.control_button_layout1.addWidget(self.duration_label)
        self.control_button_layout2.addWidget(self.topics_combo_label)
        self.control_button_layout2.addWidget(self.topics_combo_box)
        self.control_button_layout2.addWidget(self.winsize_spinbox_label)
        self.control_button_layout2.addWidget(self.windowSize_spinBox)
        self.control_button_layout2.addWidget(self.overlap_combo_label)
        self.control_button_layout2.addWidget(self.overlap_combo_box)
        self.control_button_layout3.addWidget(self.tab_container)
        self.control_button_layout3.addWidget(self.tree_of_topics)
        self.control_button_layout4.addWidget(self.taggeds_label)
        self.control_button_layout4.addWidget(self.logWindowsTagged)
        self.control_button_layout4.addWidget(self.logOutput_label)
        self.control_button_layout4.addWidget(self.logOutput)
        self.controlEnabled = False


        for t_name in self.tabs_labels.keys():
            if len(self.tabs_labels[t_name]):
                logger.debug(t_name)

        self.label_groupbox_style = " \
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

        self.listOftaggedWindows = [] # keeps track of the windows that were tagged by the user.
        self.label_group_boxes = dict([])
        self.label_button_groups = dict([])
        self.label_options = dict([])
        self.label_layouts = dict([])

        for t_name in self.tabs_labels.keys():
            if len(self.tabs_labels[t_name]):
                options_buttons = []
                button_group_boxes = dict([])
                buttons_by_labels = dict([])
                button_layouts = dict([])
                button_groups = dict([])
                for label in self.tabs_labels[t_name].keys():
                    button_group_boxes[label] = QGroupBox(title=label)
                    button_group_boxes[label].setFlat(True)
                    button_group_boxes[label].setStyleSheet(self.label_groupbox_style)
                    for opt in self.tabs_labels[t_name][label]:
                        options_buttons.append(QRadioButton(opt))
                    options_buttons[0].setChecked(True) # set a default
                    buttons_by_labels[label] = options_buttons
                    options_buttons = []
                    button_layouts[label] = QVBoxLayout() # Radio buttons usually are in a vertical layout
                    button_groups[label] = QButtonGroup() # Create a button group for radio buttons
                self.label_options[t_name] = buttons_by_labels
                self.label_layouts[t_name] = button_layouts
                self.label_button_groups[t_name] = button_groups
                self.label_group_boxes[t_name] = button_group_boxes

        for t_name in self.label_options.keys():
            for label in self.label_layouts[t_name].keys():
                #print len(self.label_options[t_name][label]), label
                for i in range(len(self.label_options[t_name][label])):
                    # Add each radio button to the button layout
                    self.label_layouts[t_name][label].addWidget(self.label_options[t_name][label][i])
                    # Add each radio button to the button group & give it an ID of i
                    self.label_button_groups[t_name][label].addButton(self.label_options[t_name][label][i], i)


        for t_name in self.label_options.keys():
            for label in self.label_layouts[t_name]:
                # Set the layout of the group box to the button layout
                self.label_group_boxes[t_name][label].setLayout(self.label_layouts[t_name][label])
                self.annotation_layouts[t_name].addWidget(self.label_group_boxes[t_name][label])


        self.tag_buttons = dict([])
        self.tag_buttons_layout = dict([])
        self.tag_buttons_vlayouts = dict([])
        for t in self.label_group_boxes.keys():
            #TODO: Remove multiple tag buttons in the different tabs. Keep one for all!!!
            self.tag_buttons_layout[t] = QHBoxLayout()
            self.tag_buttons_layout[t].setContentsMargins(0, 0, 0, 0)
            self.tag_buttons[t] = QPushButton("Tag")
            self.tag_buttons[t].clicked.connect(self.handleTag)
            self.tag_buttons[t].setEnabled(False)
            self.tag_buttons_layout[t].addWidget(self.tag_buttons[t])

            self.tag_buttons_vlayouts[t] = QVBoxLayout()
            self.tag_buttons_vlayouts[t].addLayout(self.annotation_layouts[t])
            self.tag_buttons_vlayouts[t].addLayout(self.tag_buttons_layout[t])

            self.tabs[t].setLayout(self.tag_buttons_vlayouts[t])
            self.tab_container.addTab(self.tabs[t], t)

        layout = QVBoxLayout()
        layout.addWidget(self.videoWidget)
        layout.addLayout(self.control_button_layout1)
        layout.addLayout(self.control_button_layout2)
        layout.addLayout(self.control_button_layout3)
        layout.addLayout(self.control_button_layout4)
        self.setLayout(layout)

        self.mediaPlayer.setVideoOutput(self.videoWidget.videoSurface())
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)
        self.mediaPlayer.setNotifyInterval(1)

    def addToTree(self, tree, dictionary):
        if isinstance(dictionary, dict):
            for k, v in dictionary.iteritems():
                if v == []:
                    child = QTreeWidgetItem(tree)
                    child.setFlags(child.flags() | Qt.ItemIsUserCheckable)
                    child.setText(0,k)
                    child.setCheckState(0, Qt.Unchecked)
                else:
                    parent = QTreeWidgetItem(tree)
                    parent.setText(0, k)
                    parent.setFlags(parent.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
                    self.addToTree(parent, v)


    def getTreeSelection(self,subroot, dictionary):
        if subroot.childCount():
            logger.debug("Decending: " + subroot.text(0))
            for i in range(subroot.childCount()):
                parent = subroot.child(i)
                newDict = {}
                dictionary[parent.text(0)] = self.getTreeSelection(parent,newDict)
        else:
            if subroot.checkState(0) == QtCore.Qt.Checked:
                dictionary = "ON"
                logger.debug("Set ON")
            else:
                dictionary = "OFF"
                logger.debug("Set OFF")

        return dictionary

    def hasSelectedItemOnTree(self):
        checked = dict()
        hasOneChecked = False
        if len(self.listOftaggedWindows):
            self.errorMessages(9)
        else:

            root = self.tree_of_topics.invisibleRootItem()
            signal_count = root.childCount()

            for i in range(signal_count):
                signal = root.child(i)
                checked_sweeps = list()
                num_children = signal.childCount()

                for n in range(num_children):
                    child = signal.child(n)

                    if child.checkState(0) == QtCore.Qt.Checked:
                        checked_sweeps.append(child.text(0))
                        hasOneChecked = True
                checked[signal.text(0)] = checked_sweeps

        return hasOneChecked

    #Print out the ID & text of the checked radio button
    def handleTag(self):
        logger.debug("In the hangle tag")
        if not len(self.topics_to_save) and not self.hasSelectedItemOnTree():
            self.errorMessages(10)
        else:
            if not len(self.listOftaggedWindows):
                self.topics_to_save = self.getTreeSelection(self.tree_of_topics.invisibleRootItem(),self.topics_to_save)
                self.tree_of_topics.setEnabled(False)
                self.tree_of_topics.expandAll()
                logger.debug(self.topics_to_save)

            for t_name in self.label_group_boxes.keys():
                tag_data = {}
                for label in self.label_options[t_name].keys():
                    for i in range(len(self.label_options[t_name][label])):
                        if self.label_options[t_name][label][i].isChecked():
                            #logger.debug(self.label_button_groups[t_name][label].checkedId())
                            tag_data[label] = self.label_options[t_name][label][i].text()
                current_windows = int(self.windows_combo_box.currentText())

                if current_windows in self.listOftaggedWindows:         # if an annotation for the windows already exists
                    msg = "An anottation was already given to this window of data for the '" \
                          +str(t_name) + "' tab. Do you want to overwrite it?"
                    reply = QMessageBox.question(self, 'Confirm Overwrite',msg, QMessageBox.Yes, QMessageBox.No)
                    if reply == QMessageBox.Yes:
                        self.data[t_name]["tags"][current_windows] = [tag_data[l] for l in self.data[t_name]["labels"]]
                        logger.info("Annotation for windows " + str(current_windows) + " was overwriten successfuly!")
                        self.isUnsave = True
                        if not self.filename == "":
                            self.setWindowTitle('*' + self.filename + '-' + __file__)
                        else:
                            self.setWindowTitle('* UNTITLED '+ '-' + __file__)
                        # self.csv_writers[t_name].writerows([tag_data])
                        # self.output_data_files[t_name].flush()
                    else:
                        pass
                else:
                    self.data[t_name]["tags"].append([tag_data[l]for l in self.data[t_name]["labels"]])
                    self.listOftaggedWindows.append(current_windows)
                    self.isUnsave = True
                    if not self.filename == "":
                        self.setWindowTitle('*' + self.filename + '-' + __file__)
                    else:
                        self.setWindowTitle('* UNTITLED ' + '-' + __file__)
                    logger.debug("Annotation for windows " + str(current_windows) + "done!")
                    #self.csv_writers[t_name].writerows([tag_data])
                    #self.output_data_files[t_name].flush()


        self.logWindowsTagged.setText(str(self.listOftaggedWindows))

    # processes the change in spinner windowsSize element
    def windowSizeChanged(self):
        self.wsize_value = self.windowSize_spinBox.value()
        logger.info("Windows size set to:" + str(self.windowSize_spinBox.value()))

    def windowsComboxChanged(self):
        curr = self.windows_combo_box.currentText()
        if curr != '':
            curr = int(curr)
            if self.isBagLoaded and curr == 0:
                self.previousDWindowButton.setEnabled(False)
            elif self.isBagLoaded and curr == (self.number_of_windows - 1):
                self.nexstDWindowButton.setEnabled(False)
            elif self.isBagLoaded:
                self.previousDWindowButton.setEnabled(True)
                self.nexstDWindowButton.setEnabled(True)

            millis = self.windows[curr][0]*1000
            self.updateSliderPosition(millis)
        logger.debug("Changed to new window")

    #Listens to the change in the overlap dropdown list
    def overlapComboxChanged(self, i):
        self.w_overlap_value = int(self.overlap_combo_box.currentText()[:-1])
        logger.info("Overlap value set to: "+ self.overlap_combo_box.currentText())

    # Listens to the change in the combobox dropdown list
    def topicsComboxChanged(self, i):
        self.current_image_topic = self.topics_combo_box.currentText()
        logger.info("Image topic defaulted to: " + self.current_image_topic)

    def loadOutputFiles(self):
        for t_name in self.label_group_boxes.keys():
            t_details = {}
            t_details["tags"] = [[] for n in range(self.number_of_windows)]
            t_details["labels"] = tuple([label for label in self.tabs_labels[t_name].keys()])
            t_details["win_size"] = self.wsize_value
            t_details["overlap"] = self.w_overlap_value
            t_details["image_topic"] = self.current_image_topic
            self.data[t_name] = t_details

    def reload(self):
        self.windows_combo_box.clear()
        self.topics_to_save = {}
        self.types = {}
        self.tree_of_topics.clear()
        self.loadImageTopic(self.topics_combo_box.currentText())

    def openFile(self):
        global imageBuffer,framerate
        self.bagfileName,_ = QFileDialog.getOpenFileName(self, "Open Bag", QDir.currentPath(),"*.bag")
        if self.bagfileName != '':
            if not self.bagfileName:
                pass
            else:
                try:
                    self.bag = rosbag.Bag(self.bagfileName)
                except:
                    self.errorMessages(0)

                #Get bag metadata
                (self.message_count,self.duration, self.topics,
                 self.compressedImageTopics,compressed, framerate) = get_bag_metadata(self.bag)
                #Buffer the rosbag, boxes, timestamps

                logger.info("TOPICS FOUND:") #TODO: try catch the case where theres no topics. Potential fatal error.
                for top in self.topics:
                    logger.info("\t- " + top["topic"] + "\n\t\t-Type: "+
                                                       top["type"]+"\n\t\t-Fps: "+ str(top["frequency"]))
                logger.info("BAG TOTAL DURATION: " + str(self.duration))
                if len(self.compressedImageTopics):
                    self.topics_combo_box.addItems(self.compressedImageTopics)
                    self.reloadButton.setEnabled(True)
                else:
                    self.errorMessages(6)
            self.isBagLoaded = True
            self.reload()

    def process_windows(self):
        if self.w_overlap_value:
            self.win_phase = (self.wsize_value*self.w_overlap_value)/100.0       ##Determined by cross-multiplication
        else:
            self.win_phase = self.wsize_value           #TODO: is there a better way to have 0 overlap?ssss
        counter = 0.0

        self.windows = []
        while (counter < self.time_buff_secs[-1] and counter+self.wsize_value < self.time_buff_secs[-1]):
            self.windows.append((counter, counter+self.wsize_value))     #Tuple-0: Begining - Tuple-1:End
            counter += self.win_phase
        logger.info("B: " + str([self.windows[i][0] for i in range(len(self.windows))]))
        logger.info("E: " + str([self.windows[i][1] for i in range(len(self.windows))]))
        logger.info("Size: " + str(len(self.windows)))

        self.number_of_windows = len(self.windows)
        self.listOftaggedWindows = []
        self.windows_combo_box.clear()
        for w in range(self.number_of_windows):
            self.windows_combo_box.addItem(str(w))

        self.loadOutputFiles()
        self.output_data_files = {}
        self.csv_writers = {}
        for t_name in self.label_group_boxes.keys():
            self.output_data_files[t_name] = open(t_name + ".csv", 'wa')
            self.csv_writers[t_name] = csv.DictWriter(self.output_data_files[t_name], self.data[t_name]["labels"])
            self.csv_writers[t_name].writeheader()
            logger.debug("CsVHeaders: " + str(self.data[t_name]["labels"]))

        self.logWindowsTagged.setText(str(self.listOftaggedWindows))
        self.topics_to_save = {}
        self.tree_of_topics.setEnabled(True)

    def loadImageTopic(self, topic_name):
        (imageBuffer, self.time_buff_secs) = self.buffer_data(self.bag, image_topic=topic_name)
        logger.warn(self.time_buff_secs)
        self.process_windows()
        fourcc = cv2.VideoWriter_fourcc('X', 'V' ,'I', 'D')
        height, width, bytesPerComponent = imageBuffer[0].shape
        video_writer = cv2.VideoWriter(self.bagfileName[:-4]+".avi", fourcc, framerate, (width,height), cv2.IMREAD_COLOR)

        if not video_writer.isOpened():
            self.errorMessages(2)
        else:
            for frame in imageBuffer:
                video_writer.write(frame)
            video_writer.release()

            self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(self.bagfileName[:-4]+".avi")))
            self.playButton.setEnabled(True)
            self.previousDWindowButton.setEnabled(True)
            self.nexstDWindowButton.setEnabled(True)
            self.saveButton.setEnabled(True)
            for b in self.tag_buttons.keys():
                self.tag_buttons[b].setEnabled(True)

    def buffer_data(self, bag, image_topic, compressed=True):
        img_buff = []
        img_time_buff_secs = []
        start_time = None
        bridge = CvBridge()
        self.bag_buffers = {}

        for t_name in [top["topic"] for top in self.topics]:
            self.bag_buffers[t_name] = {}
            self.bag_buffers[t_name]["msg"] = []
            self.bag_buffers[t_name]["s_time"] = None
            self.bag_buffers[t_name]["time_buffer_secs"] = []

        # Buffer the images, timestamps from the rosbag
        for topic, msg, t in bag.read_messages():  # topics=[input_topic]):

            if self.bag_buffers[topic]["s_time"] is None:
                self.bag_buffers[topic]["s_time"] = t

            # Get the image
            if topic == image_topic:
                if not compressed:
                    try:
                        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError as e:
                        print e
                else:
                    nparr = np.fromstring(msg.data, np.uint8)
                    cv_image = cv2.imdecode(nparr, 1)  #### cv2.CV_LOAD_IMAGE_COLOR has as enum value 1.
                    ## TODO: fix the problem with the this enum value.

                img_buff.append(cv_image)
                img_time_buff_secs.append(t.to_sec() - self.bag_buffers[topic]["s_time"].to_sec())

            self.bag_buffers[topic]["msg"].append(msg)
            self.bag_buffers[topic]["time_buffer_secs"].append(t.to_sec() - self.bag_buffers[topic]["s_time"].to_sec())

        self.types = {}
        dictionary = {}
        for k in self.bag_buffers.keys():
            if k not in self.compressedImageTopics:
                logger.debug(10*'-'+' ' +k)
                if not self.bag_buffers[k]["msg"][0]._type in self.types:
                    self.types[self.bag_buffers[k]["msg"][0]._type] = self.makeTopicDictionary(self.bag_buffers[k]["msg"][0],dictionary)
                    dictionary = {}

        #self.addToTree(self.tree_of_topics, self.dat)
        self.addToTree(self.tree_of_topics,self.types)
        logger.debug(json.dumps(self.types, indent=4, sort_keys=True))
        return img_buff, img_time_buff_secs

    def isPrimitive(self,obj):
        """ __slots__ gives the list of fields in the msg. It a message doesn't have it,
        it is reasonable to say it doesn't contain fields on it, thus, it is a primitive
        type"""
        return not hasattr(obj, '__slots__')

    def makeTopicDictionary(self,root, dictionary):
        if not self.isPrimitive(root):
            for s in root.__slots__:
                if not s.startswith("header"):
                    if self.isPrimitive(getattr(root,s)):
                        dictionary = self.makeTopicDictionary(s, dictionary)
                    else:
                        newDict ={}
                        dictionary[s] = self.makeTopicDictionary(getattr(root,s),newDict)
        else:
            dictionary[root] = []

        return dictionary


    #Open CSV file
    def openCsv(self):
        fileName, _ = QFileDialog.getOpenFileName(self, "Open Csv ", QDir.currentPath(), "*.csv")
        box_buff, metrics_buff, box_action = buffer_csv(fileName)



    def parseConfig(self):
        with open("config.json") as json_file:
                json_data = json.load(json_file)
        return json_data

    def errorMessages(self,index):
        msgBox = QMessageBox()
        msgBox.setIcon(QMessageBox.Critical)
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
        elif index == 6:
            msgBox.setText("Error: Bag file has no compressed image topics!")
        elif index == 7:
            msgBox.setText("Error: something went in the windows partition!")
        elif index == 8:
            msgBox.setText('Error: Manually moving the slider is NOT allowed in this version. '
                           'Use the "Window" drop-down list!')
        elif index == 9:
            msgBox.setText('You cannot change the target features after loading the bag file!')
        elif index == 10:
            msgBox.setText('You must select at least one topic in the check box tree! '
                           'Tip: check the item but also click on its name')
        msgBox.resize(100,40)
        msgBox.exec_()

    def play(self):
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.mediaPlayer.pause()
        else:
            self.mediaPlayer.play()

    def moveWindowForward(self):
        curr = self.windows_combo_box.currentText()
        if curr != '':
            curr = self.windows_combo_box.findText(curr)
            if (curr+1) < self.number_of_windows:
                self.windows_combo_box.setCurrentIndex(curr+1)
                self.windowsComboxChanged()

    def moveWindowBackward(self):
        curr = self.windows_combo_box.currentText()
        if curr != '':
            curr = self.windows_combo_box.findText(curr)
            if (curr-1) >= 0:
                self.windows_combo_box.setCurrentIndex(curr-1)
                self.windowsComboxChanged()

    def mediaStateChanged(self, state):
        if state == QMediaPlayer.PlayingState:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        self.duration_label.setText(str((int)((position / 1000) / 60)).zfill(2) + ":" + str((int)(position / 1000) % 60).zfill(2))
        if float((position / 1000) % 60) >= self.windows[int(self.windows_combo_box.currentText())][1]:
            #logger.debug("BegW: " + str(self.windows[int(self.windows_combo_box.currentText())][0]))
            #logger.debug("EndW: " + str(self.windows[int(self.windows_combo_box.currentText())][1]))
            #logger.debug("CurEnd:" + str(float((position / 1000) % 60)))
            #logger.debug(40*'%')
            millis = self.windows[int(self.windows_combo_box.currentText())][0] * 1000
            self.updateSliderPosition(millis)
        #else: logger.debug("Cur:" + str(float((position / 1000) % 60)))
        self.positionSlider.setValue(position)

    def keyPressEvent(self,event):
        if event.key() == Qt.Key_Control:
            self.controlEnabled = True

    def keyReleaseEvent(self,event):
        if event.key() == Qt.Key_Control:
            self.controlEnabled = False

    def durationChanged(self, duration):
        self.positionSlider.setRange(0, duration)

    def updateSliderPosition(self, position):
        global frameCounter
        frameCounter = int(round(self.message_count * position / (self.duration * 1000)))
        self.mediaPlayer.setPosition(position)

    def setPosition(self):
        self.errorMessages(8)

    def saveAs(self):
        defaultdir = os.path.dirname(os.path.abspath(__file__))
        defaultname = self.bagfileName.split("/")[-1][:-4] + ".json"
        insertedName = QFileDialog.getSaveFileName(self, 'Save File', defaultdir+"/"+defaultname, filter='*.json')
        if insertedName[0] != '':
            self.filename = insertedName[0]
            logger.debug(self.filename)
            with open(self.filename, "w") as save_file:
                json.dump(self.data, save_file, indent=4, sort_keys=True)
            self.isUnsave = False
            self.setWindowTitle(self.filename + '-' + __file__)

    def save(self):
        if self.filename != '':
            with open(self.filename, "w") as save_file:
                json.dump(self.data, save_file, indent=4, sort_keys=True)
        else:
            self.saveAs()

    def closeEvent(self,event):
        if self.isBagLoaded:
            if self.listOftaggedWindows != self.number_of_windows:
                quit_msg = "Some windows were NOT annotated. Are you sure you want to exit the program?"
                reply = QMessageBox.question(self, 'Risk of data loss',
                                            quit_msg, QMessageBox.Yes, QMessageBox.No)
                if reply == QMessageBox.Yes:
                    event.accept()
                else:
                    event.ignore()



if __name__ == '__main__':
    app = QApplication(sys.argv)

    player = VideoPlayer()
    player.resize(1340,1500)
    player.show()

    sys.exit(app.exec_())

