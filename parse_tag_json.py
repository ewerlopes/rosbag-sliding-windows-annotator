#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import copy
import rosbag
import traceback
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
        self.annotationDictionary = {}     # Topics present on the annotated file.
        self.topicSelection = {}           # This loads the type of objects in the treeviewer. Used for saying which topic to save.
        self.topicSelectionON = {}         # holds which items in the tree of topics is ON
        self.annotationFileName = ''
        self.windowsInterval = []          # stores the start and end times for the windows.
        self.bagFileName = ''
        self.isAnnotationReady = False
        self.isBagReady = False
        self.bag_topics = {}               # topics as extracted from the bag info.
        self.bag_data   = {}               # as a list extracted from the bag_topics
        self.csv_writers = {}              # the csv file objects
        self.output_data_files = {}        # the data file names.


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
        self.save_label = QLabel("Saving to:")

        # Create text area for files loaded.
        self.bagFileTextArea = QTextEdit()
        self.bagFileTextArea.setReadOnly(True)
        self.bagFileTextArea.setLineWrapMode(QTextEdit.NoWrap)
        self.bagFileTextArea.setMaximumHeight(50)

        # Save text area
        self.saveTextArea = QTextEdit()
        self.saveTextArea.setReadOnly(True)
        self.saveTextArea.setLineWrapMode(QTextEdit.NoWrap)
        self.saveTextArea.setMaximumHeight(50)

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
        body_layout.addWidget(self.save_label)
        body_layout.addWidget(self.saveTextArea)
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
        for k,v in self.topicSelection.iteritems():
            if v == "ON":
                selectionParts = k.split("")
                featureName = selectionParts[0]
                if self.topicSelection[features]
                self.topicSelectionON[featureName].append("".join(selectionParts[1:]))


        logger.info("Tree selection: \n" + json.dumps(self.topicSelection, indent=4, sort_keys=True))
        logger.debug("Selected (ON): "+ json.dumps(self.topicSelectionON,
                                                   indent=4, sort_keys=True))

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
                self.addToTree(self.tree_of_topics, self.annotationDictionary["topics"])
            except:
                self.errorMessages(5)

            if self.mustCheckCompatibility():
                if self.areFileCompatible():
                    self._setAnnotationFlags()
                else:
                    self.errorMessages(1)
                    logger.error("Could not load" + self.annotationFileName + " annotation file! "
                                 "Reason: bag is incompatible with the given annotation file.")
                    self.annotationFileName = ''
            else:
                self._setAnnotationFlags()


    def _setAnnotationFlags(self):
        self.loadWindowsTime()
        self.isAnnotationReady = True
        if self.isEnableSave():
            self.saveButton.setEnabled(True)

    def _setBagFlags(self):
        self.bagFileTextArea.setText(self.bagFileName)
        self.isBagReady = True
        if self.isEnableSave():
            self.saveButton.setEnabled(True)

    def loadWindowsTime(self):
        str_buffer = ["\nLOADED WINDOWS INTERVAL"]
        for i,w in enumerate(self.annotationDictionary["windows_interval"]):
            self.windowsInterval.append((w[0],w[1]))
            str_buffer.append("\t#"+str(i)+" - Start: " + str(w[0]) + "secs\t|\tEnd:" + str(w[1]) + "secs")
        total_bag_time = self.annotationDictionary["duration"]
        total_win_time = self.annotationDictionary["windows_interval"]                                      \
                                                  [len(self.annotationDictionary["windows_interval"])-1]    \
                                                  [1]
        str_buffer.append("TOTAL BAG DURATION: "+ str(total_bag_time))
        str_buffer.append("TOTAL WINDOWING TIME: "+str(total_win_time))
        str_buffer.append("TIME NOT USED: " + str(float((total_bag_time)-float(total_win_time))))
        logger.info("\n".join(str_buffer))


    def openBagFile(self):
        self.bagFileName, _ = QFileDialog.getOpenFileName(self, "Open Bag", QDir.currentPath(), "*.bag")
        if self.bagFileName != '':
            try:
                self.bag = rosbag.Bag(self.bagFileName)
                info_dict = yaml.load(self.bag._get_yaml_info())
                self.bag_topics = info_dict['topics']

                string_buffer = []
                string_buffer.append("\nTOPICS FOUND:\n")
                # TODO: try catch the case where there's no topics, currently a potential fatal error.
                for top in self.bag_topics:
                    string_buffer.append("\t- " + top["topic"] + "\n\t\t-Type: " +
                                         top["type"] + "\n\t\t-Fps: " + str(top["frequency"]))

                logger.info("\n".join(string_buffer))
            except Exception,e:
                self.errorMessages(4)
                logger.error(str(e))
            if self.mustCheckCompatibility():
                if self.areFileCompatible():
                    self._setBagFlags()
                else:
                    self.errorMessages(0)
                    logger.error("Could not load" + self.bagFileName +" the bag file! "
                                 "Reason: bag is incompatible with the given annotation file.")
                    self.bagFileName = ''
                    self.bag = ''
                    self.bag_data = ''
                    self.bag_topics = ''
            else:
                self._setBagFlags()

    def mustCheckCompatibility(self):
        if self.isBagReady and self.isAnnotationReady:
            return True
        else: return False

    def areFileCompatible(self):
        """Checks if the jason file cam be used in the current loaded bag. In other words,
        whether it has the topics the jason file lists under the key 'topics'"""
        bagOftopics = []
        # Checking if the topic is compressed
        for d in self.annotationDictionary["topics"].keys():
            if d not in [top["topic"] for top in self.bag_topics]:
                return False
        return True


    def getBagData(self):
        self.bag_data = {}

        for t_name in [top["topic"] for top in self.bag_topics]:
            self.bag_data[t_name] = {}
            self.bag_data[t_name]["msg"] = []
            self.bag_data[t_name]["s_time"] = None
            self.bag_data[t_name]["time_buffer_secs"] = []

        # Buffer the images, timestamps from the rosbag
        for topic, msg, t in self.bag.read_messages(topics=[top["topic"] for top in self.bag_topics]):
            try:
                if self.bag_data[topic]["s_time"] == None:
                    self.bag_data[topic]["s_time"] = t


                self.bag_data[topic]["msg"].append(msg)
                self.bag_data[topic]["time_buffer_secs"].append(t.to_sec() -
                                                                 self.bag_data[topic]["s_time"].to_sec())
            except:
                logger.debug("Error: " + topic)


        #self.topicSelection
        #logger.debug(json.dumps(self.bag_data, indent=4))

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
            msgBox.setText("Error: You must load a bag file and/or an annotation file!")
        elif index == 4:
            msgBox.setText("Error: Error when opening the bag file!")
        elif index == 5:
            msgBox.setText("Error: Error when opening the annotation json file!")
        msgBox.resize(100,40)
        msgBox.exec_()

    def printDataToFileOLD(self):
        try:
            for s_name in self.annotationDictionary["sources"]:
                for item in self.topicSelectionON:
                    topicList =  item.split(".")
                    topic = topicList[0]
                    logger.debug("Splited: " + topic)
                    for t,w in enumerate(self.windowsInterval):
                        start = w[0]
                        end = w[1]
                        buffer = []
                        index_s = 0
                        for i in range(len(self.bag_data[topic]["time_buffer_secs"])):
                            if self.bag_data[topic]["time_buffer_secs"][i] >= start:
                                index_s = i
                                break
                        for j in range(index_s,len(self.bag_data[topic]["time_buffer_secs"]) - index_s):
                            if self.bag_data[topic]["time_buffer_secs"][j] <= end:
                                if self.annotationDictionary[s_name]["tags"][t] == []:
                                    self.csv_writers[s_name].writerows([{}])
                                else:
                                    row = copy.copy(self.annotationDictionary[s_name]["tags"][t])
                                    logger.debug("TAG: " + str(self.annotationDictionary[s_name]["tags"][t]))
                                    logger.debug("Copy: "+ str(row))
                                    logger.debug("ITEM: "+item)
                                    logger.debug("Command: " + ".".join(topicList[1:]))
                                    row[item] = getattr(self.bag_data[topic]["msg"][j],".".join(topicList[1:]))
                                    if row[item] == None:
                                        logger.debug("NONE!!!!")
                                    logger.debug("Row: "+ str(row))
                                    buffer.append(row)
                        self.csv_writers[s_name].writerows(buffer)
                        self.csv_writers[s_name].writerows([{}])
                        self.output_data_files[s_name].flush()
        except Exception as e:
            logger.error(traceback.format_exc())

    def printDataToFile(self):
        try:
            t_time_buffer_sizes = {}
            for t_name in [top["topic"] for top in self.bag_topics]:
                t_time_buffer_sizes[t_name] = len(self.bag_data[t_name]["time_buffer_secs"])

            for s_name in self.annotationDictionary["sources"]:
                for t,w in enumerate(self.windowsInterval):
                    if self.annotationDictionary[s_name]["tags"][t] == []:
                        self.csv_writers[s_name].writerows([{}])
                    else:
                        start = w[0]
                        end = w[1]
                        buffer = []
                        index_s = 0
                        row = copy.copy(self.annotationDictionary[s_name]["tags"][t])
                        for topic, tBufferSize in t_time_buffer_sizes.iteritems():
                            for index in range(tBufferSize):
                                if self.bag_data[topic]["time_buffer_secs"][index] >= start:
                                    index_s = index
                                    break
                            for j in range(index_s,(tBufferSize-index_s)):
                                if self.bag_data[topic]["time_buffer_secs"][j] <= end:
                                    logger.debug("TAG: " + str(self.annotationDictionary[s_name]["tags"][t]))
                                    logger.debug("Copy: "+ str(row))
                                    logger.debug("ITEM: "+ topic)
                                    logger.debug("Command: " + self.topicSelectionON[topic])
                                    row[topic] = getattr(self.bag_data[topic]["msg"][j],self.topicSelectionON[topic])
                                    if row[topic] == None:
                                        logger.debug("NONE!!!!")
                                    logger.debug("Row: "+ str(row))
                                    buffer.append(row)
                            self.csv_writers[s_name].writerows(buffer)
                            self.csv_writers[s_name].writerows([{}])
                            self.output_data_files[s_name].flush()
        except Exception as e:
            logger.error(traceback.format_exc())

    def save(self):
        self.processTreeOfTopics()
        if self.isEnableSave() and self.treeHasItemSelected():
            self.getBagData()
            defaultdir = os.path.dirname(os.path.abspath(__file__))
            defaultname = self.bagFileName.split("/")[-1][:-4] + ".csv"
            insertedName = QFileDialog.getSaveFileName(self, 'Save File', defaultdir + "/"
                                                       + defaultname, filter='*.csv')
            if insertedName[0] != '':
                if insertedName[0].endswith(".csv"):
                    filename = insertedName[0][:-4]
                    self.saveTextArea.setText(filename)
                else:
                    filename = insertedName[0]
                    self.saveTextArea.setText(filename)

                for s_name in self.annotationDictionary["sources"]:
                    self.output_data_files[s_name] = open(filename+"_"+s_name + ".csv", 'wa')
                    self.csv_writers[s_name] = csv.DictWriter(self.output_data_files[s_name],
                                                              self.annotationDictionary[s_name]["labels"]+
                                                              self.topicSelectionON)
                    self.csv_writers[s_name].writeheader()
                    self.output_data_files[s_name].flush()

                self.printDataToFile()

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

