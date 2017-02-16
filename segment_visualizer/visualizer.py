#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""The Video Widget example shows how to implement a video widget
using QtMultimedia's QAbstractVideoSurface.

The following is a translation into PyQt5 from the C++ example found in
C:\QtEnterprise\5.1.1\msvc2010\examples\multimediawidgets\customvideosurface\customvideowidget."""
from __future__ import division
import rosbag

import json
import random
import matplotlib
import pickle
import pandas
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from view_utils import *

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
        self.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.MinimumExpanding)
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
        global frameCounter

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


class PandasModel(QtCore.QAbstractTableModel):
    def __init__(self, data, parent=None):
        QtCore.QAbstractTableModel.__init__(self, parent)
        self._data = data

    def rowCount(self, parent=None):
        return len(self._data.values)

    def columnCount(self, parent=None):
        return self._data.shape[1]

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return list(self._data)[section]
        return QAbstractTableModel.headerData(self, section, orientation, role)

    def data(self, index, role=Qt.DisplayRole):
        if index.isValid():
            if role == Qt.DisplayRole:
                return QtCore.QVariant(str(
                    self._data.values[index.row()][index.column()]))
        return QtCore.QVariant()


class VideoPlayer(QWidget):
    def __init__(self, parent=None):
        global gantChart
        super(VideoPlayer, self).__init__(parent)
        self.videobox = []
        self.metric_buffer = []
        self.filename = ''
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        self.WINDOWS_MISMATCH_TOLERANCE = 0.01      #tolerance for mismatch in the windows endpoint.
        self.current_begin_mismatch = -1

        # the jason config data for setting labels
        self.label_configs = self.parseConfig()
        self.data = {}
        self.types = {}          # This loads the type of objects in the treeviewer. Used for saying which topic to save.
        self.dataset_info = "./segments.pkl"

        fileObject = open(self.dataset_info, 'r')
        # load the object from the file into var b
        self.frame = pickle.load(fileObject)
        self.dataset_size = self.frame.shape[0]
        self.frame['tag'] = ""
        self.isBagLoaded = False
        self.model_data = None
        self.bag_dir = '/home/ewerlopes/developer/rosbag-sliding-windows-annotator/all_tagged_2017_01_18'
        self.alignment_begin_at = None # where to begin playing on the time line.
        self.alignment_discrete_duration = None
        self.bagfileName = None #the bag file currently on display

        #### MATPLOTLIB #####
        # a figure instance to plot on
        self.figure = plt.figure()
        #self.figure2 = plt.figure()
        # this is the Canvas Widget that displays the `figure`
        # it takes the `figure` instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)
        #self.canvas2 = FigureCanvas(self.figure2)
        # this is the Navigation widget
        # it takes the Canvas widget and a parent
        self.toolbar = NavigationToolbar(self.canvas, self)

        # set the layout
        self.matplot_layout = QVBoxLayout()
        self.matplot_layout.addWidget(self.toolbar)
        self.matplot_layout.addWidget(self.canvas)
        #self.matplot_layout.addWidget(self.canvas2)
        #######

        # video widget
        self.videoWidget = VideoWidget()

        # Buttons
        self.openButton = QPushButton("Load cluster data")
        self.openButton.clicked.connect(self.openFile)

        self.saveButton = QPushButton("Save Progress")
        self.saveButton.setEnabled(False)
        self.saveButton.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.saveButton.clicked.connect(self.handleSave)
        self.isUnsave = False
        self.ntag_performed = 0

        self.playButton = QPushButton("Play")
        self.playButton.setEnabled(False)
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.play)


        self.nexstDWindowButton = QPushButton("Next")
        self.nexstDWindowButton.setEnabled(False)
        self.nexstDWindowButton.setIcon(self.style().standardIcon(QStyle.SP_MediaSkipForward))
        self.nexstDWindowButton.clicked.connect(self.moveWindowForward)

        self.previousDWindowButton = QPushButton("Previous")
        self.previousDWindowButton.setEnabled(False)
        self.previousDWindowButton.setIcon(self.style().standardIcon(QStyle.SP_MediaSkipBackward))
        self.previousDWindowButton.clicked.connect(self.moveWindowBackward)

        self.reloadButton = QPushButton("Reload")
        self.reloadButton.setEnabled(False)
        self.reloadButton.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        self.reloadButton.clicked.connect(self.reload)

        self.delTagButton = QPushButton("Delete Tag")
        self.delTagButton.setEnabled(False)
        self.delTagButton.setIcon(self.style().standardIcon(QStyle.SP_DialogCancelButton))
        self.delTagButton.clicked.connect(self.handleRemoveTag)

        ### VIDEO SLIDER ###
        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setRange(0, 0)
        self.positionSlider.sliderPressed.connect(self.setPosition)

        self.repeat_cbox = QCheckBox()
        self.repeat_cbox.setGeometry(QtCore.QRect(50, 390, 71, 21))
        self.repeat_cbox.setObjectName('repeat')

        ### TREE OF BAG TOPICS
        self.tree_of_topics = QTreeWidget()
        self.tree_of_topics.setHeaderLabel("Topics")
        #self.tree_of_topics.setMaximumWidth(500)

        ### BUTTONS FOR THE SECOND CONTROL BUTTON LAYOUT

        # Create a label widget for buttons in the second layout
        self.topics_combo_label = QLabel('Image Topics:')
        self.logOutput_label = QLabel("Log area:")
        self.duration_label = QLabel("--:--")
        self.repeat_cbox_label = QLabel("set_repeat")
        self.save_label = QLabel("Save to:")
        self.percen_annotated = QLabel('0% tagged!')

        #Create overlap dropdown list
        self.files_combo_box = QComboBox()
        self.files_combo_box.currentIndexChanged.connect(self.filesComboxChanged)

        # Create overlap dropdown list
        self.topics_combo_box = QComboBox()
        self.topics_combo_box.currentIndexChanged.connect(self.topicsComboxChanged)

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

        # Create log area
        self.save = QTextEdit()
        self.save.setMaximumHeight(22)
        self.save_scroll = self.save.verticalScrollBar()
        self.save_scroll.setValue(self.save_scroll.minimum())
        self.save_font = self.save.font()
        self.save_font.setFamily("Courier")
        self.save_font.setPointSize(14)
        self.save.moveCursor(QTextCursor.End)
        self.save.setCurrentFont(self.save_font)
        self.save.document().setPlainText("annotation.pkl")

        ### REDIRECT CONSOLE OUTPUT ####
        XStream.stdout().messageWritten.connect(self.logOutput.append)
        XStream.stderr().messageWritten.connect(self.logOutput.append)

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


        self.control_button_layout0 = QHBoxLayout()
        self.control_button_layout1 = QHBoxLayout()
        self.control_button_layout2 = QHBoxLayout()
        self.control_button_layout3 = QVBoxLayout()
        self.control_button_layout4 = QVBoxLayout()

        self.tab_layout = QHBoxLayout()
        self.control_panel_layout = QHBoxLayout()


        #### Control panel buttons ###
        self.tableWidget = QTableView()

        self.table_model = PandasModel(self.frame[['x', 'y', 'z', 'file', 'align', 'tag']])
        self.tableWidget.doubleClicked.connect(self.on_click)

        self.by_files_button = QRadioButton('Select File')
        self.by_files_button.toggled.connect(self.byFileRButtonHandle)
        self.by_files_button.setChecked(True)

        self.all_button = QRadioButton('All')
        self.all_button.toggled.connect(self.allRButtonHandle)

        self.control_panel_groupbox = QGroupBox(title='Control panel')
        self.control_panel_groupbox.setFlat(True)
        self.control_panel_groupbox.setStyleSheet(self.label_groupbox_style)
        self.control_panel_groupbox.setLayout(self.control_panel_layout)
        self.cpanel_internal_layout1 = QVBoxLayout()
        self.cpanel_internal_layout1child = QHBoxLayout()
        self.cpanel_internal_layout2child = QVBoxLayout()

        self.control_button_layout0.addWidget(self.videoWidget)
        self.control_button_layout0.addLayout(self.matplot_layout)
        self.control_button_layout1.setContentsMargins(0, 0, 0, 0)
        #self.control_button_layout1.addWidget(self.openButton)
        self.control_button_layout1.addWidget(self.reloadButton)
        #self.control_button_layout1.addWidget(self.repeat_cbox_label)
        #self.control_button_layout1.addWidget(self.repeat_cbox)
        self.control_button_layout1.addWidget(self.positionSlider)
        self.control_button_layout1.addWidget(self.duration_label)
        #self.control_button_layout2.addWidget(self.previousDWindowButton)
        self.control_button_layout2.addWidget(self.saveButton)
        self.control_button_layout2.addWidget(self.playButton)
        #self.control_button_layout2.addWidget(self.nexstDWindowButton)
        self.control_button_layout2.addWidget(self.topics_combo_label)
        self.control_button_layout2.addWidget(self.topics_combo_box)
        self.control_button_layout3.addWidget(self.control_panel_groupbox)
        self.control_button_layout4.addWidget(self.logOutput_label)
        self.control_button_layout4.addWidget(self.logOutput)
        self.tab_layout.addWidget(self.tab_container)
        self.cpanel_internal_layout1child.addWidget(self.all_button)
        self.cpanel_internal_layout1child.addWidget(self.by_files_button)
        #self.cpanel_internal_layout1child.addWidget(self.files_combox_label)
        self.cpanel_internal_layout1child.addWidget(self.files_combo_box)
        self.cpanel_internal_layout2child.addWidget(self.save_label)
        self.cpanel_internal_layout2child.addWidget(self.save)
        #self.control_panel_layout.addWidget(self.tree_of_topics)
        self.control_panel_layout.addLayout(self.tab_layout)
        self.cpanel_internal_layout2child.addWidget(self.tableWidget)
        self.cpanel_internal_layout2child.addWidget(self.percen_annotated)
        self.cpanel_internal_layout2child.addWidget(self.delTagButton)
        self.cpanel_internal_layout1.addLayout(self.cpanel_internal_layout1child)
        self.cpanel_internal_layout1.addLayout(self.cpanel_internal_layout2child)
        self.control_panel_layout.addLayout(self.cpanel_internal_layout1)

        self.controlEnabled = False

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
        layout.addLayout(self.control_button_layout0)
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

    def createFileTable(self, file_name):
        self.model_data = self.frame[['x', 'y', 'z', 'file', 'align', 'tag']].loc[self.frame['file'] == file_name]
        self.tableWidget.setModel(PandasModel(self.model_data))
        self.tableWidget.show()

    def createClusterTable(self):
        self.model_data = self.frame[['x', 'y', 'z', 'file', 'align', 'tag']]
        self.tableWidget.setModel(PandasModel(self.model_data))
        self.tableWidget.show()

    def on_click(self):
        index = self.tableWidget.selectedIndexes()[0]
        index = index.row()
        data = self.model_data.iloc[[index]]
        #print data
        self.plotSegment(data['x'].tolist()[0],data['y'].tolist()[0],data['z'].tolist()[0])
        bag_to_open = data['file'].tolist()[0].split('.')[0]
        bag_to_open = bag_to_open.split('_')[:2]
        self.alignment_begin_at = data['align'].tolist()[0]
        self.alignment_discrete_duration = len(data['z'].tolist()[0])

        if self.bagfileName != os.path.join(self.bag_dir,'_'+"_".join(bag_to_open)+'.bag'):
            self.openFile(os.path.join(self.bag_dir,'_'+"_".join(bag_to_open)+'.bag'))
        self.updateSliderPosition(self.bag_buffers['robogame/imu_state']['time_buffer_secs'][self.alignment_begin_at]*1000)
        #self.plotTopicData()

    def allRButtonHandle(self):
        self.files_combo_box.setEnabled(False)
        self.createClusterTable()

    def byFileRButtonHandle(self):
        files_list = set(self.frame['file'])
        for s in files_list:
            self.files_combo_box.addItem(s)
        self.files_combo_box.setEnabled(True)
        self.createFileTable(str(self.files_combo_box.currentText()))

    def plotSegment(self, dx, dy, dz):
        ''' plot the segment data '''

        # create an axis
        ax = self.figure.add_subplot(111)
        ax.clear()
        
        # plot data
        ax.plot(dx,'r*-' , label='x')
        ax.plot(dy,'b.-' , label='y')
        ax.plot(dz,'g+-' , label='z')
        ##ax.set_ylim(-0.1,0.1)
        ax.set_xlabel('Number of samples')
        #ax.set_ylabel('magnitude')
        ax.grid()
        ax.legend()

        # refresh canvas
        self.canvas.draw()

    def plotTopicData(self):
        ''' plot some topic data '''

        # create an axis
        ax = self.figure2.add_subplot(111)

        # discards the old graph
        ax.hold(False)

        # plot data
        xAxisData = self.bag_buffers['robogame/imu_state']['msg'][
                    self.alignment_begin_at:self.alignment_discrete_duration]
        ax.plot([d.linear_acc.x for d in xAxisData], '*-')
        ax.set_ylim(-0.1,0.1)
        ax.set_xlabel('Number of samples')
        ax.set_ylabel('Jerk')

        # refresh canvas
        self.canvas2.draw()

    def addToTree(self, tree, dictionary):
        if isinstance(dictionary, dict):
            for k, v in dictionary.iteritems():
                if v == []:
                    child = QTreeWidgetItem(tree)
                    #child.setFlags(child.flags() | Qt.ItemIsUserCheckable)
                    child.setText(0,k)
                    #child.setCheckState(0, Qt.Unchecked)
                else:
                    parent = QTreeWidgetItem(tree)
                    parent.setText(0, k)
                    #parent.setFlags(parent.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
                    #parent.setFlags(parent.flags() | Qt.ItemIsTristate)
                    self.addToTree(parent, v)


    def getTreeSelection(self,subroot, dictionary):
        if subroot.childCount():
            for i in range(subroot.childCount()):
                parent = subroot.child(i)
                newDict = {}
                dictionary[parent.text(0)] = self.getTreeSelection(parent,newDict)
        else:
            if subroot.checkState(0) == QtCore.Qt.Checked:
                dictionary = "ON"
            else:
                dictionary = "OFF"
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

    def handleSave(self):
        file_obj = open(self.save.toPlainText(), 'wb')
        pickle.dump(self.frame, file_obj)
        file_obj.close()
        self.isUnsave = False
        self.saveButton.setText('Save Progress')
        logger.info('Changes saved to file.')

    def handleRemoveTag(self):
        #print tag_data
        index = self.tableWidget.selectedIndexes()[0]
        index = index.row()
        d = self.model_data.iloc[[index]]
        self.frame.loc[d.index.tolist()[0], 'tag'] = ""
        self.model_data = self.frame[['x', 'y', 'z', 'file', 'align', 'tag']].loc[self.frame['file'] == d['file'].tolist()[0]]
        self.tableWidget.setModel(PandasModel(self.model_data))
        self.tableWidget.selectRow(index)
        self.tableWidget.show()
        self.isUnsave = True
        if self.ntag_performed != 0:
            self.ntag_performed -= 1
        self.saveButton.setText('Save Progress*')
        self.percen_annotated.setText('{:.2f} % tagged!'.format((self.ntag_performed*100)/self.dataset_size))
        self.percen_annotated.setStyleSheet('color:green')

    #Print out the ID & text of the checked radio button
    def handleTag(self):
        for t_name in self.label_group_boxes.keys():
            tag_data = {}
            for label in self.label_options[t_name].keys():
                for i in range(len(self.label_options[t_name][label])):
                    if self.label_options[t_name][label][i].isChecked():
                        #logger.debug(self.label_button_groups[t_name][label].checkedId())
                        tag_data[label] = self.label_options[t_name][label][i].text()

        #print tag_data
        index = self.tableWidget.selectedIndexes()[0]
        index = index.row()
        d = self.model_data.iloc[[index]]
        self.frame.loc[d.index.tolist()[0], 'tag'] = str(tag_data)
        self.model_data = self.frame[['x', 'y', 'z', 'file', 'align', 'tag']].loc[self.frame['file'] == d['file'].tolist()[0]]
        self.tableWidget.setModel(PandasModel(self.model_data))
        self.tableWidget.selectRow(index)
        self.tableWidget.show()
        self.isUnsave = True
        self.ntag_performed += 1
        self.saveButton.setText('Save Progress*')
        self.percen_annotated.setText('{:.2f} % tagged!'.format((self.ntag_performed*100)/self.dataset_size))
        self.percen_annotated.setStyleSheet('color:green')


        #     logger.debug(json.dumps(tag_data,indent=4))
        #
        #     if current_windows in self.listOftaggedWindows:         # if an annotation for the windows already exists
        #         msg = "An anottation was already given to this window of data for the '" \
        #               +str(t_name) + "' tab. Do you want to overwrite it?"
        #         reply = QMessageBox.question(self, 'Confirm Overwrite',msg, QMessageBox.Yes, QMessageBox.No)
        #         if reply == QMessageBox.Yes:
        #             self.data[t_name]["tags"][current_windows] = tag_data#[tag_data[l] for l in self.data[t_name]["labels"]]
        #             logger.info("ANNOTATION FOR WINDOWS " + str(current_windows) + " <- OVERWRITTEN!")
        #             self.isUnsave = True
        #             if not self.filename == "":
        #                 self.setWindowTitle('*' + self.filename + '-' + __file__)
        #             else:
        #                 self.setWindowTitle('* UNTITLED '+ '-' + __file__)
        #     else:
        #         self.data[t_name]["tags"][current_windows] = tag_data
        #         self.listOftaggedWindows.append(current_windows)
        #         self.listOftaggedWindows.sort()
        #         if not self.filename == "":
        #             self.setWindowTitle('*' + self.filename + '-' + __file__)
        #         else:
        #             self.setWindowTitle('* UNTITLED ' + '-' + __file__)
        #         logger.info("ANNOTATION FOR WINDOWS " + str(current_windows) + " <-- DONE!")
        #
        # self.logWindowsTagged.setText(str(self.listOftaggedWindows))

    #Listens to the change in the overlap dropdown list
    def filesComboxChanged(self, i):
        self.createFileTable(str(self.files_combo_box.currentText()))

    # Listens to the change in the combobox dropdown list
    def topicsComboxChanged(self, i):
        self.current_image_topic = self.topics_combo_box.currentText()
        logger.info("Image topic defaulted to: " + self.current_image_topic)

    def reload(self):
        self.reset()

    def reset(self):
        """Unconditionally reset the environment"""
        self.types = {}
        self.tree_of_topics.clear()
        self.loadImageTopic(self.topics_combo_box.currentText())

    def openFile(self,filename):
        global imageBuffer,framerate
        self.bagfileName = filename #QFileDialog.getOpenFileName(self, "Open Bag", QDir.currentPath(),"*.bag")
        #logger.info(self.bagfileName)
        print self.bagfileName
        
        if self.bagfileName != '':
            if not self.bagfileName:
                pass
            else:
                try:
                    self.bag = rosbag.Bag(self.bagfileName)
                except Exception as e:
                    print str(e)
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
            self.reset()

    def loadImageTopic(self, topic_name):
        (imageBuffer, self.time_buff_secs) = self.buffer_data(self.bag, image_topic=topic_name)
        self.topics_to_save = {}
        self.tree_of_topics.setEnabled(True)
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
            self.saveButton.setEnabled(True)
            self.delTagButton.setEnabled(True)
            for b in self.tag_buttons.keys():
                self.tag_buttons[b].setEnabled(True)

    def buffer_data(self, bag, image_topic, compressed=True):
        img_buff = []
        img_time_buff_secs = []
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
                if not self.bag_buffers[k]["msg"][0]._type in self.types:
                    self.types[self.bag_buffers[k]["msg"][0]._type] = self.makeTopicDictionary(self.bag_buffers[k]["msg"][0],dictionary)
                    dictionary = {}


        dictionary = {}
        for top in self.topics:
            for type in self.types.keys():
                if top["type"] == type:
                    dictionary[top["topic"]] = self.types[type]

        self.data["topics"] = dictionary
        self.addToTree(self.tree_of_topics,self.data["topics"])
        #logger.debug(json.dumps(self.types, indent=4, sort_keys=True))
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
            msgBox.setText("Error: Video could not be initialized")
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
        pass

    def moveWindowBackward(self):
        pass

    def mediaStateChanged(self, state):
        if state == QMediaPlayer.PlayingState:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        self.duration_label.setText(str((int)((position / 1000) / 60)).zfill(2) + ":" + str((int)(position / 1000) % 60).zfill(2))

        if float(position / 1000) >= float(self.bag_buffers['robogame/imu_state']['time_buffer_secs'][self.alignment_begin_at+self.alignment_discrete_duration]):
            millis = self.bag_buffers['robogame/imu_state']['time_buffer_secs'][self.alignment_begin_at]*1000
            self.updateSliderPosition(millis)
        else:
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

    def closeEvent(self,event):
        if self.isBagLoaded:
            if self.isUnsave:
                quit_msg = "Unsaved changes exist! Do you want to save them before exit?"
                reply = QMessageBox.question(self, 'Risk of data loss',
                                            quit_msg, QMessageBox.Yes, QMessageBox.No)
                if reply == QMessageBox.Yes:
                    event.accept()
                    self.handleSave()
                else:
                    event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)

    player = VideoPlayer()
    player.resize(600, 400)
    player.show()

    sys.exit(app.exec_())

