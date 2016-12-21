#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Ewerton Lopes
# Politecnico di Milano, December, 2016.
# This program aims at exporting bag data into csv files. This piece of
# software should be used as an extension of the "annotation.py" program (also
# included in this directory). It should receive the bag file and the annotation
# generated as a json file by the annotation.py. Then, it exports the data into
# a csv that can then be used easily by other platforms.

import json
import copy
import rosbag
import traceback
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from annotator_utils import *
from collections import defaultdict

### logging setup #####
logger = logging.getLogger(__name__)
handler = QtHandler()
format = '%(asctime)s -- %(levelname)s --> %(message)s'
date_format = '%Y-%m-%d %H:%M:%S'
handler.setFormatter(logging.Formatter(format,date_format))
logger.addHandler(handler)
logger.setLevel(logging.DEBUG)
#######################

class AnnotationParser(QWidget):
    def __init__(self, parent=None):
        super(AnnotationParser, self).__init__(parent)

        # the jason config data for setting labels
        self.bag = ''                       # The loaded bag object
        self.annotationDictionary = {}      # Variable to hold the data from the annotation file.
        self.topicSelectionState = {}       # This loads the status of the topics in the tree viewer.
                                            # Used for saying which topic to save.

        self.topicSelectionON = defaultdict(list)   # holds which items in the tree of topics is ON
        self.topicSelectionONHeaders = []           # holds the topic names as collapsed from the dict.
                                                    # nested variables have their attribute name separated
                                                    # and attached to the topic name. It is then used as
                                                    # the name of each column in the csv file.

        self.annotationFileName = ''                # Holds the name of the json loaded file
        self.windowsInterval = []          # stores the list of tuples representing the windows interval as
                                           # loaded from the json. Each item in the list is a tuple (x,y)
                                           # where the x is the start time and y is the end time for the windows.

        self.bagFileName = ''              # holds the bag file name.
        self.isAnnotationReady = False     # flag to say whether the json file is loaded.
        self.isBagReady = False            # flag to say whether the bag file is loaded.
        self.bag_topics = {}               # topics as extracted from the bag info.
        self.bag_data   = {}               # a dictionary holding information loaded from the topics. it
                                           # comprises for each topic: msg (list), time_buffer in secs (list)
                                           # and the starting time.
        self.csv_writers = {}              # the dictionary of csv write objects.
        self.output_filenames = {}         # the csv file objects.
        self.mismatchTolerance = 0.02     # tolerance value for deviation in the windows slice


        # Create Push Buttons
        self.openButton = QPushButton("Open Bag")
        self.openButton.clicked.connect(self.openBagFile)
        self.openButton.setIcon(self.style().standardIcon(QStyle.SP_DialogOpenButton))
        self.openButton.setMinimumWidth(250)

        # Loaded Push Button.
        self.loadTagJsonButton = QPushButton("Load annotation")
        self.loadTagJsonButton.clicked.connect(self.openAnnotationFile)
        self.loadTagJsonButton.setIcon(self.style().standardIcon(QStyle.SP_FileIcon))
        self.loadTagJsonButton.setMaximumWidth(350)

        # Export Push Button
        self.exportButton = QPushButton("Export CSV")
        self.exportButton.setEnabled(False)
        self.exportButton.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.exportButton.setMaximumWidth(300)
        self.exportButton.clicked.connect(self.exportCSV)

        # Creat spinner box (windows size)
        self.tolerance_spinbox = QDoubleSpinBox()
        self.tolerance_spinbox.valueChanged.connect(self.setMismatchTolerance)
        self.tolerance_spinbox.setValue(0.02)
        self.tolerance_spinbox.setSingleStep(0.01)
        self.tolerance_spinbox.setMaximumWidth(100)
        self.tolerance_spinbox.setToolTip("Mismatch Tolerance: Deviation from the jason and "
                                          "bag start/end endpoint")

        # Create tree widget for listing the topic names
        self.tree_of_topics = QTreeWidget()
        self.tree_of_topics.setHeaderLabel("Topics")

        # Create a labels
        self.logOutput_label = QLabel("Log area:")
        self.export_label = QLabel("Exported to:")
        self.mismatch_label = QLabel("Mismatch tolerance: ")
        self.mismatch_label.setMaximumHeight(50)

        # Create text area for files loaded.
        self.bagFileTextArea = QTextEdit()
        self.bagFileTextArea.setReadOnly(True)
        self.bagFileTextArea.setLineWrapMode(QTextEdit.NoWrap)
        self.bagFileTextArea.setMaximumHeight(50)

        # "Exported to" text area
        self.exportTextArea = QTextEdit()
        self.exportTextArea.setReadOnly(True)
        self.exportTextArea.setLineWrapMode(QTextEdit.NoWrap)
        self.exportTextArea.setMaximumHeight(50)

        # Create log area
        self.annotationFileTextArea = QTextEdit()
        self.annotationFileTextArea.setReadOnly(True)
        self.annotationFileTextArea.setLineWrapMode(QTextEdit.NoWrap)
        self.annotationFileTextArea.setMaximumHeight(50)

        # Create log area
        self.logOutput = QTextEdit()
        self.logOutput.setReadOnly(True)
        self.logOutput.setLineWrapMode(QTextEdit.WidgetWidth)

        # Set font type and size for the log area
        self.log_font = self.logOutput.font()
        self.log_font.setFamily("Courier")
        self.log_font.setPointSize(10)

        # Set the log are properties: color, font and cursor movement
        self.logOutput.moveCursor(QTextCursor.End)
        self.logOutput.setCurrentFont(self.log_font)
        self.logOutput.setTextColor(QColor("red"))

        # Set log area scroll bar properties
        self.scroll_bar = self.logOutput.verticalScrollBar()
        self.scroll_bar.setValue(self.scroll_bar.maximum())

        # Redirecting the output strem to the logOutput area.
        XStream.stdout().messageWritten.connect(self.logOutput.append)
        XStream.stderr().messageWritten.connect(self.logOutput.append)

        # Layouts
        self.control_layout1 = QHBoxLayout()
        self.control_layout2 = QHBoxLayout()
        self.control_layout3 = QHBoxLayout()
        self.control_layout1.addWidget(self.openButton)
        self.control_layout1.addWidget(self.bagFileTextArea)
        self.control_layout2.addWidget(self.loadTagJsonButton)
        self.control_layout2.addWidget(self.annotationFileTextArea)
        self.control_layout3.addWidget(self.exportButton)
        self.control_layout3.addWidget(self.mismatch_label)
        self.control_layout3.addWidget(self.tolerance_spinbox)
        self.control_layout3.setAlignment(Qt.AlignLeft)

        # Defining the whole main windows body layout.
        body_layout = QVBoxLayout()
        body_layout.addLayout(self.control_layout1)
        body_layout.addLayout(self.control_layout2)
        body_layout.addWidget(self.tree_of_topics)
        body_layout.addLayout(self.control_layout3)
        body_layout.addWidget(self.export_label)
        body_layout.addWidget(self.exportTextArea)
        body_layout.addWidget(self.logOutput_label)
        body_layout.addWidget(self.logOutput)
        self.setLayout(body_layout)

    def reset(self):
        """Reset variables"""
        self.topicSelectionState = {}
        self.topicSelectionON = defaultdict(list)
        self.topicSelectionONHeaders = []

    def generateTreeOfTopics(self, tree, dictionary):
        """Creates the buttons of the tree of topics."""
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
                    self.generateTreeOfTopics(parent, v)

    def parseTreeOfTopics(self, subroot, dictionary):
        """Recursive Method that process the tree of loaded topics,
        identifying which of the topics are checked and which are not.

        subroot  :   the parent topic in the three (typically the name of the topic,
                    also being an attribute name if the topic has a nested data type.
                    Normally, The first parent is the root of the tree widget (in PyQt), that is
                    the tree.invisibleRootItem().
        dictionary  : the data structure (dict) used for encoding the result, i.e., which of
                    the elements are ON or OFF.

        NOTE that this is a recursive method that checks the state ON/OFF of the checked
        tree buttons, so it supports nested data types encoding the structure in a python dict.
        """

        # Checks whether the subroot has children.
        if subroot.childCount():
            # In case it has children, loop over them.
            for i in range(subroot.childCount()):
                parent = subroot.child(i)
                newDict = {}
                #recursively call the method passing the child of the subroot as a new parent.
                dictionary[parent.text(0)] = self.parseTreeOfTopics(parent, newDict)
        # Base case, where the subroot has no children. It is a primitive item.
        else:
            #set the subroot state depending whether it is checked or not.
            if subroot.checkState(0) == QtCore.Qt.Checked:
                dictionary = "ON"
            else:
                dictionary = "OFF"
        # return the dictionary for completing the recursivity.
        return dictionary

    def flatten_dict(self, dd, separator='.', prefix=''):
        """This function collapses a dictionary into a list, by appending
        the keys' values to themselves. That is, parents(keys) are joined together
        with children (values) by the separator variable.

        dd  :   dictionary to be flattened
        separator   :   the character used to join values to their keys.
        prefix      :   the character used in place of the value.
        """
        return {prefix + separator + k if prefix else k: v
                for kk, vv in dd.items()
                for k, v in self.flatten_dict(vv, separator, kk).items()
                } if isinstance(dd, dict) else {prefix: dd}

    def loadSelectedTopics(self):
        """Method that identifies which of the topics are selected (marked as ON),
        creating a dictionary variable indexed by the name of the topic and
        whose values describe the attributes of that topic."""

        self.topicSelectionState = self.flatten_dict(
                                    self.parseTreeOfTopics(self.tree_of_topics.invisibleRootItem(),
                                                           self.topicSelectionState))
        for k,v in self.topicSelectionState.iteritems():
            if v == "ON":
                selectionParts = k.split(".")
                featureName = selectionParts[0]
                # sets the dictionary whose keys describes the topic name and values
                # are attributes of the topic type.
                self.topicSelectionON[featureName].append(".".join(selectionParts[1:]))
                # defines a list of topics that are set as ON.
                self.topicSelectionONHeaders.append(k)

        logger.info("Tree selection: \n" + json.dumps(self.topicSelectionState, indent=4, sort_keys=True))
        logger.info("Selected (ON): "+ json.dumps(self.topicSelectionON,
                                                   indent=4, sort_keys=True))
        logger.info("Selected (ON) Headers: " + json.dumps(self.topicSelectionONHeaders,
                                                    indent=4, sort_keys=True))

    def treeHasItemSelected(self):
        """Checks whether there is at least ONE topic marked as ON"""
        hasOne = False
        for k,v in self.topicSelectionState.iteritems():
            if v == "ON":
                hasOne = True
                break
        return hasOne

    def isExportEnable(self):
        """Checks whether the saving button can be activated and,
        thus, enabling the csv to be generated."""
        return self.isBagReady and self.isAnnotationReady

    def openAnnotationFile(self):
        """Prompts the user for loading the json file. This function is
        used by the "load annotation" button. Works pretty much at the same
        way of openBagFile method. See that function for more details."""
        self.annotationFileName, _ = QFileDialog.getOpenFileName(self, "Open", QDir.currentPath(), "*.json")
        if self.annotationFileName != '':
            try:
                self.annotationDictionary = self.loadJson(self.annotationFileName)
                self.annotationFileTextArea.setText(self.annotationFileName)
                self.generateTreeOfTopics(self.tree_of_topics, self.annotationDictionary["topics"])
            except:
                self.errorMessages(5)

            if self.mustCheckCompatibility():
                if self.areFileCompatible():
                    self.setAnnotationFlags()
                else:
                    self.errorMessages(1)
                    logger.error("Could not load" + self.annotationFileName + " annotation file! "
                                 "Reason: bag is incompatible with the given annotation file.")
                    self.annotationFileName = ''
            else:
                self.setAnnotationFlags()

    def setAnnotationFlags(self):
        """Sets the flag for marking whether the json file was loaded."""
        self.loadAnnotationData()
        self.isAnnotationReady = True
        if self.isExportEnable(): # Checks whether it is time for enabling the export button.
            self.exportButton.setEnabled(True)

    def setMismatchTolerance(self):
        """A callback function for setting the tolerance for the mismatch
        between the json start/end and bag start/end endpoints."""
        self.mismatchTolerance = self.tolerance_spinbox.value()

    def setBagFlags(self):
        """Sets the flag for marking whether the bag file was loaded."""
        self.bagFileTextArea.setText(self.bagFileName) # sets the name of the textEdit
        self.isBagReady = True
        if self.isExportEnable():     # Checks whether it is time for enabling the export button.
            self.exportButton.setEnabled(True)

    def loadAnnotationData(self):
        """Load data pointed out by the json file: Windows timing (begin and
         end intervals) for each windows, and bag time duration"""
        # set a variable for string buffering. Used for printing detail about the bag duration
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
        """Prompts the user for choosing the bag file and loads the data."""
        self.bagFileName, _ = QFileDialog.getOpenFileName(self, "Open Bag", QDir.currentPath(), "*.bag")

        # in case the bagFileName returned by the windows is not empty. That is, the
        # user did not canceled the opening of the file (situation where we get an empty name)
        if self.bagFileName != '':
            try:
                #Read the bag.
                self.bag = rosbag.Bag(self.bagFileName)
                #Load the bag info into a dictionary.
                info_dict = yaml.load(self.bag._get_yaml_info())
                # store the topics.
                self.bag_topics = info_dict['topics']

                ### PRINT BAG INFO ###
                string_buffer = []
                string_buffer.append("\nTOPICS FOUND:\n")
                # TODO: try catch the case where there's no topics, currently a potential fatal error.
                for top in self.bag_topics:
                    string_buffer.append("\t- " + top["topic"] + "\n\t\t-Type: " +
                                         top["type"] + "\n\t\t-Fps: " + str(top["frequency"]))
                logger.info("\n".join(string_buffer))
                #######

            except Exception,e:
                self.errorMessages(4)
                logger.error(str(e))

            # Checks whether the compatibility with the json has to be checker. That is,
            # in case the json is already loaded, we need to check whether this bag file
            # has the topics the json refers to.
            if self.mustCheckCompatibility():
                if self.areFileCompatible():
                    self.setBagFlags()
                else:
                    # In case areFileCompatible method returns false, we reset the
                    # variables this function set.
                    self.errorMessages(0)
                    logger.error("Could not load" + self.bagFileName +" the bag file! "
                                 "Reason: bag is incompatible with the given annotation file.")
                    self.bagFileName = ''
                    self.bag = ''
                    self.bag_data = ''
                    self.bag_topics = ''
            # If we don't need to check compatibility (the bag is the only file loaded),
            # confirm the bag by setting the bagFlags using the _setBagFlags.
            else:
                self.setBagFlags()

    def mustCheckCompatibility(self):
        """A boolean method and return true if both the bag and the annotation jason files
        had been loaded. Returns false otherwise. The idea of creating this function is to
        avoid the use of repetitive checking condition in the code and to allow unordered
        loading of the files. That is, it does not matter each one of the files had been
         loaded first."""
        if self.isBagReady and self.isAnnotationReady:
            return True
        else: return False

    def areFileCompatible(self):
        """Checks if the jason file can be used in the current loaded bag. In other words,
        whether it has the topics the jason file lists under the key 'topics'. Note that
        if the bag file is different from the one the json file holds the tagged data, but
        has the topics, this function is going to be positive to the compatibility. This is
        a potential situation for error. In other words, it allows to use the json file
        created from other bag in a totally different one, given that it has the listed bags."""

        #loops through the list of topic names listed in the jason and returns false
        # when it sees a topic that is not in the current loaded bag file.
        for d in self.annotationDictionary["topics"].keys():
            if d not in [top["topic"] for top in self.bag_topics]:
                return False
        return True

    def loadBagData(self):
        """Sets the bag_data dictionary with with the content of the
        loaded bag.
            self.bag_data[topicName]["msg"] : list of msgs in the bag for the
                                              the given topic (topicName).
            self.bag_data[topiName]["s_time"] : time of the first msg in the
                                                bag for the given topic
            self.bag_data[topicName]["time_buffer_secs"] : list of msg arrival times (in secs)
                                                            for the given bag.
        """
        self.bag_data = {}

        for t_name in [top["topic"] for top in self.bag_topics]:
            # define msg structure. See method stringdoc.
            self.bag_data[t_name] = {}
            self.bag_data[t_name]["msg"] = []
            self.bag_data[t_name]["s_time"] = None
            self.bag_data[t_name]["time_buffer_secs"] = []

        # Buffer the images, timestamps from the rosbag
        for topic, msg, t in self.bag.read_messages(topics=[top["topic"] for top in self.bag_topics]):
            try:
                if self.bag_data[topic]["s_time"] == None:
                    self.bag_data[topic]["s_time"] = t      # sets initial time for the topic s_time.


                self.bag_data[topic]["msg"].append(msg)             # append msg
                # append second difference between the current time ant the s_time.
                self.bag_data[topic]["time_buffer_secs"].append(t.to_sec() -
                                                                 self.bag_data[topic]["s_time"].to_sec())
            except:
                logger.debug("Error: " + topic)

    def loadJson(self, filename):
        """Loads a json. Returns its content in a dictionary"""
        with open(filename) as json_file:
                json_data = json.load(json_file)
        return json_data

    def errorMessages(self,index):
        """Defines error messages via index parameter"""
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

    def writeData(self):
        """This function loops through the self.bag_data["msg] data list and based on
        the windows division (self.windowsInterval), prints the data the csv file."""

        logger.info("Aligning different time buffers...")
        # getting a combined timeline using the topics timebuffers.
        self.timeline = {}              # combined time line
        self.sorted_timeline = {}       # the sorted combined time line (it is necessary since dicts are unsorted)
        for s_name in self.annotationDictionary["sources"]:
            combined_buffer = {}
            for topicName in self.topicSelectionON.keys():
                # getting a combined timeline for all user selected topics. combined buffer
                # is a dictionary structure that saves the time(in secs) as key and each topic
                # in the given time as values. If two different topics have the same time, they
                # are stored as a list.
                [combined_buffer.setdefault(t,[]).append(topicName)
                 for t in self.bag_data[topicName]["time_buffer_secs"]]

            # saving the current combined buffer for the feature category (tabs)
            self.timeline[s_name] = combined_buffer
            # sorting the combined buffer for easing the following loops.
            self.sorted_timeline[s_name] = sorted(combined_buffer)

        try:
            # For each feature category (tabs)
            for s_name in self.annotationDictionary["sources"]:
                # Loops through all windows.
                for t,w in enumerate(self.windowsInterval):
                    logger.info("Feature Category: "+ s_name + '\tWin#: ' + str(t))
                    # skip empty tag in the jason file.
                    if self.annotationDictionary[s_name]["tags"][t] == []:
                        # print empty row to the output csv file
                        self.csv_writers[s_name].writerows([{}])
                    else:
                        start = w[0]        # start of the windows
                        end = w[1]          # end of the windows
                        buffer = []         # windows content
                        index_s = 0         # windows start index (allowing looping through the self.timeline)
                        index_e = 0         # windows end index (allowing looping through the self.timeline)

                        ##### loops to discover start index
                        for i in range(len(self.sorted_timeline[s_name])):
                            if self.sorted_timeline[s_name][i] >= start:
                                index_s = i     # set windows start index.
                                break           # exit this start index discovering loop.

                        ##### loops, getting the msg data until the windows end endpoint is reached
                        for j in range(index_s,len(self.timeline[s_name])):
                            # loops while the current index is less then or equal to the windows end endpoint
                            if self.sorted_timeline[s_name][j] <= end:
                                index_e = j     # sets the current index for the data.
                                # copy tag data from current window
                                row = copy.copy(self.annotationDictionary[s_name]["tags"][t])
                                # set the current time stamp for the row
                                row["time"] = self.sorted_timeline[s_name][index_e]
                                # calls self._getMsgValue to retrieve the data for each selected topic (topic field).
                                for topicName in self.timeline[s_name][row["time"]]:
                                    # get data. NOTE: the msg vector is aligned with the time_buffer_sec
                                    # for a given topic, this is because they are currently being saved at
                                    # the same time in getBagData() method. So, we only have to discover
                                    # which msg index is associated with the self.sorted_timeline[s_name]
                                    # value at index_e.
                                    row = self.getTopicValue(row, self.bag_data[topicName]["msg"]
                                    [self.bag_data[topicName]["time_buffer_secs"].index(row["time"])], topicName)
                                    # append row to the windows row batch
                                    buffer.append(row)
                            else:
                                break       ### spin the windows, allowing to move on.

                        ##### Checks whether the deviation between the windows "begin"
                        ##### and "end" times is less the tolerance value.
                        try:
                            assert abs(start - self.sorted_timeline[s_name][index_s]) < self.mismatchTolerance
                            logger.info("WStart: " + str(start) + " Retrieval start:" +
                                        str(self.sorted_timeline[s_name][index_s]) + " Sync: OK!")
                        except Exception as e:
                            logger.error("Beginning of the windows is out of sync! MustBe: "+
                                         str(start) + "\tWas: " + str(self.sorted_timeline[s_name][index_s]))
                        try:
                            assert abs(end -self.sorted_timeline[s_name][index_e]) < self.mismatchTolerance
                            logger.info("WEnd: " + str(end) + " Retrieval end:" +
                                        str(self.sorted_timeline[s_name][index_e]) + " Sync: OK!")
                        except Exception as e:
                            logger.error("End of the windows is out of sync! MustBe: "+
                                         str(end) + "\tWas: " +
                                         str(self.sorted_timeline[s_name][index_e]))

                        ##### Prints the windows content (row batch) to the corresponding (s_name) csv file.
                        self.csv_writers[s_name].writerows(buffer)  #write content to the file
                        self.csv_writers[s_name].writerows([{}])    #write an empty line to mark the end of the windows
                        self.output_filenames[s_name].flush()       #flush data.

        except Exception as e:
            logger.error(traceback.format_exc())

    def getTopicValue(self, dictionary, msg, parent, ignore = ["header"]):
        """Recursively saves in "dictionary" the msg values of the topics set on in the
        tree.
            dictionary : a dictionary returned by the function where the key is the name of
                         the topic, followed by its attribute names (all the way down to the
                         primitive one) and the value is the rosbag topic field msg value.
            msg:        the rosbag msg from which to extract the values.
            parent  :   the topic name from which to search the value in the rosbag msg. note
                        that it is common to have nested data types and so, the parent name is
                        used in order to describe the level of the data topic structure the re-
                        cursion is at. For example, a 6-DOF accelerometer data type may have
                        different attributes, like "gyro" and "acc" each of which may have other
                        attributes like "x","y","z".
            ignore  :   the list of ignored attributes in the msg.
        """
        # if the current topic level has attributes (i.e., it is not a primitive time)
        if hasattr(type(msg), '__slots__'):
            # loops through each attribute of this type
            for s in type(msg).__slots__:
                # ignores attribute if it is in the ignore list.
                if s in ignore:
                    continue
                else:
                    # get the attribute msg value.
                    val = msg.__getattribute__(s)
                    # call the current method again to check if the val attribute value
                    # has another attribute members.
                    dictionary = self.getTopicValue(dictionary, val, ".".join([parent, s]))
        else:
            # if the msg is of a primitive type, set it to dictionary, but only if
            # it is one of the selected topics by the user.
            if parent in self.topicSelectionONHeaders:
                dictionary[parent] = msg

        # NOTE: that if the parent (data) does not belong to the selected topics by the
        # user, the dictionary returned in the current recursion call is the same of the
        # previous one.
        return dictionary

    def exportCSV(self):
        """Opens a dialog windows and asks the general filename
        used for saving the data. It generates as much files as those
        defined in the json source field. In other words, one for each tab
        (feature perspective) used for tagging the data."""

        # Gets each topics were signed for saving.
        self.loadSelectedTopics()
        # checks whether the user has loaded the bag and the jason file and
        # has also checked at least one topic from the tree of topics.
        if self.isExportEnable() and self.treeHasItemSelected():
            # loads the bag file data in the self.bag_data dictionary variable.
            self.loadBagData()
            # defaults directory to the one where the parser program is located in
            defaultdir = os.path.dirname(os.path.abspath(__file__))
            # defaults the name of the output file(s) to the name of the bag + "csv".
            defaultname = self.bagFileName.split("/")[-1][:-4] + ".csv"
            # gets the name of the file from windows.
            insertedName = QFileDialog.getSaveFileName(self, 'Save File', defaultdir + "/"
                                                       + defaultname, filter='*.csv')
            # does nothing in case the file name is empty (the user closed the save windows
            # before pressing save button on it)
            if insertedName[0] != '':
                ###### Process the filename
                # removing .csv extension. This is done because we append to the file
                # the name of the feature perpective (tab name in the annotator.py)
                if insertedName[0].endswith(".csv"):
                    filename = insertedName[0][:-4]     #remove .csv
                else:
                    # keep the name as it is in case it has no csv extension
                    filename = insertedName[0]

                try:
                    #variable that holds the outputfiles for each perspective. Type: dictionary.
                    self.output_filenames = {}
                    #loop through perspectives.
                    for s_name in self.annotationDictionary["sources"]:
                        # append to the filename the feature perspective name
                        filename = filename + "_" + s_name + ".csv"
                        # set the "Exported to" text area
                        self.exportTextArea.append("\n" + filename)
                        # set output files
                        self.output_filenames[s_name] = open(filename, 'wa')
                        # define the headers for the csv files (variable, column, names).
                        self.csv_writers[s_name] = csv.DictWriter(self.output_filenames[s_name],
                                                                  ["time"] + self.annotationDictionary[s_name]["labels"]
                                                                  + self.topicSelectionONHeaders)
                        # write the headers
                        self.csv_writers[s_name].writeheader()
                        # flush data
                        self.output_filenames[s_name].flush()
                except Exception as e:
                    logger.error(traceback.format_exc())

                # loop through the data printing the windows content.
                self.writeData()

        # If there is no topic selected in the tree of topics before the button is pressed, ask the user
        # to select at least one.
        elif not self.treeHasItemSelected():
            self.errorMessages(2)
        # If there is no file loaded, ask the user to load them.
        elif not self.isEnabled():
            self.errorMessages(3)

        self.reset()        # reset important variable.

    def closeEvent(self,event):
        """Caputes the pressing of the windows exit button (x button)"""
        pass # currently does nothing.

if __name__ == '__main__':
    app = QApplication(sys.argv)

    player = AnnotationParser()
    player.resize(1340, QApplication.desktop().screenGeometry().height())
    player.show()

    sys.exit(app.exec_())