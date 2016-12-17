#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from annotator_utils import *

### logging setup #####
logger = logging.getLogger(__name__)
handler = QtHandler()
format = '%(asctime)s -- %(levelname)s --> %(message)s'
date_format = '%Y-%m-%d %H:%M:%S'
handler.setFormatter(logging.Formatter(format,date_format))
logger.addHandler(handler)
logger.setLevel(logging.DEBUG)

class AnnotationParser(QWidget):
    def __init__(self, parent=None):
        super(AnnotationParser, self).__init__(parent)
        self.filename = ''

        # the jason config data for setting labels
        self.data = {}
        self.types = {}          # This loads the type of objects in the treeviewer. Used for saying which topic to save.
        self.isBagLoaded = False
        self.fileName = ''
        self.isFileLoaded = False
        self.openButton = QPushButton("Open")
        self.openButton.clicked.connect(self.openFile)
        self.openButton.setIcon(self.style().standardIcon(QStyle.SP_DialogOpenButton))
        self.openButton.setMaximumWidth(150)

        self.saveButton = QPushButton("Save")
        self.saveButton.setEnabled(False)
        self.saveButton.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.saveButton.setMaximumWidth(150)
        self.saveButton.clicked.connect(self.save)

        self.tree_of_topics = QTreeWidget()
        self.tree_of_topics.setHeaderLabel("Topics")

        ### BUTTONS FOR THE SECOND CONTROL BUTTON LAYOUT
        # Create a label widget for buttons in the second layout
        self.logOutput_label = QLabel("Log area:")

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

        #XStream.stdout().messageWritten.connect(self.logOutput.append)
        #XStream.stderr().messageWritten.connect(self.logOutput.append)

        self.topics_to_save = {}


        body_layout = QVBoxLayout()
        body_layout.addWidget(self.openButton)
        body_layout.addWidget(self.tree_of_topics)
        body_layout.addWidget(self.saveButton)
        body_layout.addWidget(self.logOutput_label)
        body_layout.addWidget(self.logOutput)
        self.setLayout(body_layout)

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
                    parent.setFlags(parent.flags() | Qt.ItemIsTristate)
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

    def loadOutputFiles(self):
        for t_name in self.label_group_boxes.keys():
            t_details = {}
            t_details["tags"] = [[] for n in range(self.number_of_windows)]             # the tag you give for the window
            headers = tuple([label for label in self.tabs_labels[t_name].keys()] + ["interval_seconds"])
            t_details["labels"] =  headers #how the feature is called
            t_details["win_size"] = self.wsize_value
            t_details["overlap"] = self.w_overlap_value
            t_details["number_windows"] = self.number_of_windows
            t_details["image_topic"] = self.current_image_topic
            t_details["duration"] = self.duration
            self.data[t_name] = t_details

    def openFile(self):
        global imageBuffer,framerate
        self.fileName,_ = QFileDialog.getOpenFileName(self, "Open", QDir.currentPath(),"*.json")
        if self.fileName != '':
            try:
                self.parseJson(self.fileName)
            except:
                self.errorMessages(0)
            newTitle = self.fileName[:-5].split("/")[-1] + " - "+ __file__
            print newTitle
            #self.windowTitle(str(newTitle)) #TODO: solve the "too many argument" problem here!
            self.isFileLoaded = True

    def processContent(self):
        pass

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
        logger.info("START_TIMES: " + str([self.windows[i][0] for i in range(len(self.windows))]))
        logger.info("END_TIMES: " + str([self.windows[i][1] for i in range(len(self.windows))]))
        logger.info("NUMBER OR WINDOWS: " + str(len(self.windows)))

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
            #self.csv_writers[t_name].writeheader()

        self.logWindowsTagged.setText(str(self.listOftaggedWindows))
        self.topics_to_save = {}
        self.tree_of_topics.setEnabled(True)

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
                if not self.bag_buffers[k]["msg"][0]._type in self.types:
                    self.types[self.bag_buffers[k]["msg"][0]._type] = self.makeTopicDictionary(self.bag_buffers[k]["msg"][0],dictionary)
                    dictionary = {}

        self.addToTree(self.tree_of_topics,self.types)
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



    def parseJson(self,filename):
        with open(filename) as json_file:
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


    def saveAs(self):
        defaultdir = os.path.dirname(os.path.abspath(__file__))
        defaultname = self.bagfileName.split("/")[-1][:-4] + ".json"
        insertedName = QFileDialog.getSaveFileName(self, 'Save File', defaultdir+"/"+defaultname, filter='*.json')
        if insertedName[0] != '':
            self.filename = insertedName[0]
            with open(self.filename, "w") as save_file:
                json.dump(self.data, save_file, indent=4, sort_keys=True)
            self.isUnsave = False
            self.setWindowTitle(self.filename + '-' + __file__)

    def save(self):
        if self.filename != '':
            with open(self.filename, "w") as save_file:
                json.dump(self.data, save_file, indent=4, sort_keys=True)
            self.isUnsave = False
            self.setWindowTitle(self.filename + '-' + __file__)
        else:
            self.saveAs()

    def closeEvent(self,event):
        if self.isFileLoaded:
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

    player = AnnotationParser()
    player.resize(1340, QApplication.desktop().screenGeometry().height())
    player.show()

    sys.exit(app.exec_())

