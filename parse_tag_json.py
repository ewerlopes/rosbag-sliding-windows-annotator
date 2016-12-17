#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import rosbag
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
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
        self.bag = ''
        self.annotationDictionary = {}        # Topics present on the annotated file.
        self.topicSelection = {}          # This loads the type of objects in the treeviewer. Used for saying which topic to save.
        self.annotationFileName = ''
        self.bagFileName = ''
        self.isAnnotationReady = False
        self.isBagReady = False


        # Create Push Buttons
        self.openButton = QPushButton("Open Bag")
        self.openButton.clicked.connect(self.openBagFile)
        self.openButton.setIcon(self.style().standardIcon(QStyle.SP_DialogOpenButton))
        self.openButton.setMinimumWidth(250)

        self.loadTagJsonButton = QPushButton("Load annotation")
        self.loadTagJsonButton.clicked.connect(self.openAnnotationFile)
        self.loadTagJsonButton.setIcon(self.style().standardIcon(QStyle.SP_FileIcon))
        self.loadTagJsonButton.setMaximumWidth(350)


        self.saveButton = QPushButton("Save")
        self.saveButton.setEnabled(False)
        self.saveButton.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.saveButton.setMaximumWidth(150)
        self.saveButton.clicked.connect(self.save)

        self.tree_of_topics = QTreeWidget()
        self.tree_of_topics.setHeaderLabel("Topics")

        # Create a labels
        self.logOutput_label = QLabel("Log area:")

        # Create text area for files loaded.
        self.bagFileTextArea = QTextEdit()
        self.bagFileTextArea.setReadOnly(True)
        self.bagFileTextArea.setLineWrapMode(QTextEdit.NoWrap)
        self.bagFileTextArea.setMaximumHeight(50)

        # Create log area
        self.annotationFileTextArea = QTextEdit()
        self.annotationFileTextArea.setReadOnly(True)
        self.annotationFileTextArea.setLineWrapMode(QTextEdit.NoWrap)
        self.annotationFileTextArea.setMaximumHeight(50)

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

        self.topics_to_save = {}


        body_layout = QVBoxLayout()
        self.control_layout1 = QHBoxLayout()
        self.control_layout2 = QHBoxLayout()
        self.control_layout1.addWidget(self.openButton)
        self.control_layout1.addWidget(self.bagFileTextArea)
        self.control_layout2.addWidget(self.loadTagJsonButton)
        self.control_layout2.addWidget(self.annotationFileTextArea)
        body_layout.addLayout(self.control_layout1)
        body_layout.addLayout(self.control_layout2)
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

    def _getTreeSelection(self, subroot, dictionary):
        if subroot.childCount():
            for i in range(subroot.childCount()):
                parent = subroot.child(i)
                newDict = {}
                dictionary[parent.text(0)] = self._getTreeSelection(parent, newDict)
        else:
            if subroot.checkState(0) == QtCore.Qt.Checked:
                dictionary = "ON"
            else:
                dictionary = "OFF"
        return dictionary

    def _flatten_dict(self,dd, separator='.', prefix=''):
        return {prefix + separator + k if prefix else k: v
                for kk, vv in dd.items()
                for k, v in self._flatten_dict(vv, separator, kk).items()
                } if isinstance(dd, dict) else {prefix: dd}

    def processTreeOfTopics(self):
        self.topicSelection = self._flatten_dict(
                                    self._getTreeSelection(self.tree_of_topics.invisibleRootItem(),
                                                          self.topicSelection))
        logger.debug(json.dumps(self.topicSelection, indent=4, sort_keys=True))

    def treeHasItemSelected(self):
        hasOne = False
        for k,v in self.topicSelection.iteritems():
            if v == "ON":
                hasOne = True
                break
        return hasOne


    def isEnableSave(self):
        return self.isBagReady and self.isAnnotationReady

    def openAnnotationFile(self):
        self.annotationFileName, _ = QFileDialog.getOpenFileName(self, "Open", QDir.currentPath(), "*.json")
        if self.annotationFileName != '':
            try:
                self.annotationDictionary = self.parseJson(self.annotationFileName)
                self.annotationFileTextArea.setText(self.annotationFileName)
                newTitle = self.annotationFileName[:-5].split("/")[-1] + " - " + __file__
                self.addToTree(self.tree_of_topics, self.annotationDictionary["topics"])
            except:
                self.errorMessages(0)

            if self.mustCheckCompatibility():
                if self.areFileCompatible():
                    self.loadWindowsTime()
                    self.isAnnotationReady = True
                    # if self.isEnableSave():   #TODO: Fix this for both open methods
                    self.saveButton.setEnabled(True)
                else:
                    self.errorMessages(1)
                    logger.error("Could not load" + self.annotationFileName + " annotation file! "
                                 "Reason: bag is incompatible with the given annotation file.")
                    self.annotationFileName = ''
            else:
                self.loadWindowsTime()
                self.isAnnotationReady = True
                # if self.isEnableSave():   #TODO: Fix this for both open methods
                self.saveButton.setEnabled(True)



    def loadWindowsTime(self):
        for key in self.annotationDictionary["window_interval"]:
            self.windowInterval


    def openBagFile(self):
        self.bagFileName, _ = QFileDialog.getOpenFileName(self, "Open Bag", QDir.currentPath(), "*.bag")
        if self.bagFileName != '':
            try:
                self.bag = rosbag.Bag(self.bagFileName)
            except:
                self.errorMessages(0)

            if self.mustCheckCompatibility():
                if self.areFileCompatible():
                    self.bagFileTextArea.setText(self.bagFileName)
                    self.isBagReady = True
                    # if self.isEnableSave():   #TODO: uncomment this when able to save
                    #     self.saveButton.setEnabled(True)
                else:
                    self.errorMessages(0)
                    logger.error("Could not load" + self.bagFileName +" the bag file! "
                                 "Reason: bag is incompatible with the given annotation file.")
                    self.bagFileName = ''
            else:
                self.bagFileTextArea.setText(self.bagFileName)
                self.isBagReady = True


    def mustCheckCompatibility(self):
        if self.isBagReady and self.isAnnotationReady:
            return True
        else: return False

    def areFileCompatible(self):
        """Checks if the jason file cam be used in the current loaded bag. In other words,
        whether it has the topics the jason file lists under the key 'topics'"""
        info_dict = yaml.load(self.bag._get_yaml_info())
        topics = info_dict['topics']

        bagOftopics = []
        string_buffer = []
        string_buffer.append("\nTOPICS FOUND:\n")
        # TODO: try catch the case where there's no topics, currently a potential fatal error.
        for top in topics["topics"]:
                string_buffer.append("\t- " + top["topic"] + "\n\t\t-Type: " +
                                     top["type"] + "\n\t\t-Fps: " + str(top["frequency"]))
                string_buffer.append("BAG TOTAL DURATION: " + str(self.duration))
                logger.info("\n".join(string_buffer))
                bagOftopics.append(top["topic"])

        # Checking if the topic is compressed
        for d in self.annotationDictionary["topics"].keys():
            if d not in bagOftopics:
                return False
        return True



    def get_data(self):
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
        for topic, msg, t in self.bag.read_messages():  # topics=[input_topic]):

            if self.bag_buffers[topic]["s_time"] is None:
                self.bag_buffers[topic]["s_time"] = t


            self.bag_buffers[topic]["msg"].append(msg)
            self.bag_buffers[topic]["time_buffer_secs"].append(t.to_sec() - self.bag_buffers[topic]["s_time"].to_sec())

    def parseJson(self,filename):
        with open(filename) as json_file:
                json_data = json.load(json_file)
        return json_data

    def errorMessages(self,index):
        msgBox = QMessageBox()
        msgBox.setIcon(QMessageBox.Critical)
        if index == 0:
            msgBox.setText("Error: It was not possible to load the bag file!"
                           "Reason: topic incompatibility.")
        elif index == 1:
            msgBox.setText("Error: It was not possible to load the annotation file!"
                           "Reason: topic incompatibility.")
        elif index == 2:
            msgBox.setText("Error: You must select the topics you are interested.")
        elif index == 3:
            msgBox.setText("Error: You must load a bag file and/or an annotatio file!")
        msgBox.resize(100,40)
        msgBox.exec_()


    def save(self):
        if self.isEnableSave() and self.treeHasItemSelected():
            self.processTreeOfTopics()
            # defaultdir = os.path.dirname(os.path.abspath(__file__))
            # defaultname = self.bagFileName.split("/")[-1][:-4] + ".csv"
            # insertedName = QFileDialog.getSaveFileName(self, 'Save File', defaultdir + "/" + defaultname, filter='*.csv')
            # if insertedName[0] != '':
            #     with open(insertedName[0], "w") as save_file:
            #         #json.dump(self.data, save_file, indent=4, sort_keys=True)
            #         pass #TODO: CSV
        elif not self.treeHasItemSelected():
            self.errorMessages(2)
        elif not self.isEnabled():
            self.errorMessages(3)

    def closeEvent(self,event):
        pass



if __name__ == '__main__':
    app = QApplication(sys.argv)

    player = AnnotationParser()
    player.resize(1340, QApplication.desktop().screenGeometry().height())
    player.show()

    sys.exit(app.exec_())

