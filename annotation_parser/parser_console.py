#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Ewerton Lopes
# Politecnico di Milano, December, 2016.
# This program aims at exporting bag data as csv files. This piece of
# software should be used as an extension of the "annotation.py" program (also
# included in this directory). It should receive the bag file and the annotation
# generated as a json file by the annotation.py (for the corresponding bag file, of course).
# Then, it exports the data as a csv.


import json
import copy, os, sys, csv, yaml
import rosbag
import logging
import traceback
from collections import defaultdict, Counter

### try to load color module for logger ####
try:
    import colorlog

    have_colorlog = True
except ImportError:
    have_colorlog = False
############################################

class AnnotationConsoleParser():

    def __init__(self,bags_dir,annotation_dir, output_dir, tolerance = 0.02):

        ##### Logger setup ######
        # create logger
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)

        # create console handler and set level to debug
        ch = logging.StreamHandler(sys.__stdout__)  # Add this
        ch.setLevel(logging.DEBUG)

        # create formatter
        format = '%(asctime)s - %(levelname)-8s - %(message)s'
        date_format = '%Y-%m-%d %H:%M:%S'
        if have_colorlog and os.isatty(2):
            cformat = '%(log_color)s' + format
            formatter = colorlog.ColoredFormatter(cformat, date_format,
                                                  log_colors={'DEBUG': 'blue', 'INFO': 'green',
                                                              'WARNING': 'yellow', 'ERROR': 'red',
                                                              'CRITICAL': 'bold_red'})
        else:
            formatter = logging.Formatter(format, date_format)

        # add formatter to ch
        ch.setFormatter(formatter)
        # add ch to logger
        self.logger.addHandler(ch)
        #########################

        # setting variables
        self.bags_dir = bags_dir
        self.annotation_dir = annotation_dir
        self.output_dir = output_dir

        # set where to save
        os.mkdir(self.output_dir)

        # variable declarations
        self.mismatchTolerance = tolerance  # tolerance value for deviation in the windows slice
        self.hasTopicSelected = False
        self.allBagFiles = self.getAllFilesInDirectory(self.bags_dir, ".bag")
        self.allJsonFiles = self.getAllFilesInDirectory(self.annotation_dir, ".json")
        self.annotationDictionary = {}
        self.current_bagFile = {}
        self.current_bagOftopics = {}
        self.current_bagData = {}
        self.current_bagName = ''
        self.windowsInterval = []
        self.topicSelectionState = {}
        self.topicSelectionON = defaultdict(list)
        self.topicSelectionONHeaders = []
        self.csv_writers = {}  # the dictionary of csv write objects.
        self.output_filenames = {}  # the csv file objects.
        self.timeline = {}  # combined time line
        self.sorted_timeline = {}  # the sorted combined time line (it is necessary since dicts are unsorted)

        # the jason config data for setting labels
        self.isAutomaticTopicSelection = self.parseConfig()

    def parseConfig(self):
        """Load which topics to save from the parser_config.json. This is an effort
        to avoiding prompting the user each time a new .bag/.json pair is loaded. It
        implicitly assumes that each file in the bag_dir and annotation_dir are
        compatible. However, the compatibility is checked each time a pair of file is
        loaded.
        """
        try:
            with open("parser_config.json") as json_file:
                    config_data = json.load(json_file)
                    self.loadSelectedTopics(config_data)
        except Exception, e:
            self.logger.error(traceback.format_exc())
            self.logger.error('Failed to load "to save" topic list: ' + str(e) + ". Going manual.")
            return False
        return True

    def reset(self):
        """Reset variables"""
        self.annotationDictionary = {}
        self.windowsInterval = []
        self.current_bagFile = {}
        self.current_bagOftopics = {}
        self.current_bagData = {}
        self.current_bagName = ''
        self.timeline = {}
        self.sorted_timeline = {}
        self.csv_writers = {}  # the dictionary of csv write objects.
        self.output_filenames = {}  # the csv file objects.
        self.hasTopicSelected = False
        if not self.isAutomaticTopicSelection:
            self.topicSelectionState = {}
            self.topicSelectionON = defaultdict(list)
            self.topicSelectionONHeaders = []

    def getAllFilesInDirectory(self, directory, extension):
        """Load a list of all file names pointed in the directory variable and with the extension"""
        allFiles = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f)) and f.endswith(extension)]
        return allFiles

    def openAnnotationFile(self, jfile):
        """Loads a given json file."""
        try:
            self.annotationDictionary = self.loadJson(jfile)
            self.logger.info("Annotation file: " + jfile + " <-- LOADED!")
        except:
            self.logger.error("Error when opening annotation json file: "+jfile)
            return False
        return True

    def openBagFile(self, bfile):
        """Load a given bag file"""

        try:
            self.current_bagName = bfile
            #Read the bag.
            self.current_bagFile = rosbag.Bag(bfile)
            #Load the bag info into a dictionary.
            info_dict = yaml.load(self.current_bagFile._get_yaml_info())
            # store the topics.
            self.current_bagOftopics = info_dict['topics']

            ### PRINT BAG INFO ###
            string_buffer = []
            string_buffer.append("Bag file: "+ bfile + " <-- LOADED!")
            string_buffer.append("\nfile msg topics:\n")
            # TODO: try catch the case where there's no topics, currently a potential fatal error.
            for top in self.current_bagOftopics:
                string_buffer.append("\t- " + top["topic"] + "\n\t\t-Type: " +
                                     top["type"] + "\n\t\t-Fps: " + str(top["frequency"]))
            self.logger.info("\n".join(string_buffer))
            print ''
            #######
            return True

        except:
            self.errorMessages(4)
            self.logger.error("Filename: " + bfile + '\n' + traceback.format_exc())
            return False

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
        self.current_bagData = {}

        for t_name in [top["topic"] for top in self.current_bagOftopics]:
            # define msg structure. See method stringdoc.
            self.current_bagData[t_name] = {}
            self.current_bagData[t_name]["msg"] = []
            self.current_bagData[t_name]["s_time"] = None
            self.current_bagData[t_name]["time_buffer_secs"] = []

        # Buffer the images, timestamps from the rosbag
        for topic, msg, t in self.current_bagFile.read_messages(topics=[top["topic"] for top in self.current_bagOftopics]):
            try:
                if self.current_bagData[topic]["s_time"] == None:
                    self.current_bagData[topic]["s_time"] = t      # sets initial time for the topic s_time.


                self.current_bagData[topic]["msg"].append(msg)             # append msg
                # append second difference between the current time ant the s_time.
                self.current_bagData[topic]["time_buffer_secs"].append(t.to_sec() -
                                                                       self.current_bagData[topic]["s_time"].to_sec())
            except:
                self.logger.debug("Error: " + topic)

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

    def getUserTopicSelection(self):
        """Iteratively prompts the user which topics to save based on the annotated
        bag file topics"""

        self.topicSelectionState = {}
        for attr in self.flatten_dict(self.annotationDictionary["topics"]).keys():
            try:
                answer = raw_input("Do you want to consider '" + attr + "' in the csv? (y/n)")
            except KeyboardInterrupt:
                self.logger.info("\nProgram terminated by user request.")
                sys.exit(-1)
            if answer == 'y':
                self.hasTopicSelected = True
                self.topicSelectionState[attr] = 'ON'
                selectionParts = attr.split(".")
                featureName = selectionParts[0]
                # sets the dictionary whose keys describes the topic name and values
                # are attributes of the topic type.
                self.topicSelectionON[featureName].append(".".join(selectionParts[1:]))
                # defines a list of topics that are set as ON.
                self.topicSelectionONHeaders.append(attr)
            else:
                self.topicSelectionState[attr] = 'OFF'


        self.logger.info("Selected topics to save: \n" + json.dumps(self.topicSelectionState, indent=4, sort_keys=True))
        self.logger.debug("Selected (ON): "+ json.dumps(self.topicSelectionON,
                                                   indent=4, sort_keys=True))
        self.logger.debug("Selected (ON) Headers: " + json.dumps(self.topicSelectionONHeaders,
                                                    indent=4, sort_keys=True))

    def loadSelectedTopics(self, topics):
        """Method that identifies which of the topics are selected (marked as ON),
        creating a dictionary variable indexed by the name of the topic and
        whose values describe the attributes of that topic."""

        self.topicSelectionState = topics
        for k,v in topics.iteritems():
            if v == "ON":
                selectionParts = k.split(".")
                featureName = selectionParts[0]
                # sets the dictionary whose keys describes the topic name and values
                # are attributes of the topic type.
                self.topicSelectionON[featureName].append(".".join(selectionParts[1:]))
                # defines a list of topics that are set as ON.
                self.topicSelectionONHeaders.append(k)

        self.logger.info("Topics to save: \n" + json.dumps(self.topicSelectionState, indent=4, sort_keys=True))
        self.logger.debug("Selected (ON): "+ json.dumps(self.topicSelectionON,
                                                   indent=4, sort_keys=True))
        self.logger.debug("Selected (ON) Headers: " + json.dumps(self.topicSelectionONHeaders,
                                                    indent=4, sort_keys=True))

    def loadAnnotationData(self):
        """Load data pointed out by the json file: Windows timing (begin and
         end intervals) for each windows, and bag time duration"""
        # set a variable for string buffering. Used for printing detail about the bag duration
        str_buffer = ["\nWindows interval found:"]
        for i,w in enumerate(self.annotationDictionary["windows_interval"]):
            self.windowsInterval.append((w[0],w[1]))
            str_buffer.append("\t#"+str(i)+" - Start: " + str(w[0]) + "secs\t|\tEnd:" + str(w[1]) + "secs")
        total_bag_time = self.annotationDictionary["duration"]
        total_win_time = self.annotationDictionary["windows_interval"]                                      \
                                                  [len(self.annotationDictionary["windows_interval"])-1]    \
                                                  [1]
        str_buffer.append("\n")
        str_buffer.append("Total annotated bag duration: "+ str(total_bag_time))
        str_buffer.append("Windowing total time: "+str(total_win_time))
        str_buffer.append("Wasted time: " + str(float((total_bag_time)-float(total_win_time))))
        self.logger.info("\n".join(str_buffer))
        print ''

    def loadJson(self, filename):
        """Loads a json. Returns its content in a dictionary"""
        with open(filename) as json_file:
                json_data = json.load(json_file)
        return json_data

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
            if d not in [top["topic"] for top in self.current_bagOftopics]:
                return False
        return True

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

    def writeData(self):
        """This function loops through the self.bag_data["msg] data list and based on
        the windows division (self.windowsInterval), prints the data the csv file."""

        self.logger.info("Aligning different time buffers...")
        # getting a combined timeline using the topics timebuffers.
        self.timeline = {}  # combined time line
        self.sorted_timeline = {}  # the sorted combined time line (it is necessary since dicts are unsorted)
        for s_name in self.annotationDictionary["sources"]:
            combined_buffer = {}
            for topicName in self.topicSelectionON.keys():
                # getting a combined timeline for all user selected topics. combined buffer
                # is a dictionary structure that saves the time(in secs) as key and each topic
                # in the given time as values. If two different topics have the same time, they
                # are stored as a list.
                [combined_buffer.setdefault(t, []).append(topicName)
                 for t in self.current_bagData[topicName]["time_buffer_secs"]]

            # saving the current combined buffer for the feature category (tabs)
            self.timeline[s_name] = combined_buffer
            # sorting the combined buffer for easing the following loops.
            self.sorted_timeline[s_name] = sorted(combined_buffer)

            try:
                # assert size of lists, debugging purposes.
                assert len(self.sorted_timeline[s_name]) == len(set(self.timeline[s_name].keys()))
            except:
                self.logger.error(traceback.format_exc())
                sys.exit(-1)
        try:
            # For each feature category (tabs)
            for s_name in self.annotationDictionary["sources"]:
                # Loops through all windows.
                for t,w in enumerate(self.windowsInterval):
                    self.logger.info("***** Feature Category: "+ s_name + '\tWin#: ' + str(t))
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
                        for j in range(index_s,len(self.sorted_timeline[s_name])):
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
                                    row = self.getTopicValue(row, self.current_bagData[topicName]["msg"]
                                    [self.current_bagData[topicName]["time_buffer_secs"].index(row["time"])], topicName)
                                    # append row to the windows row batch
                                    buffer.append(row)
                            else:
                                break       ### spin the windows, allowing to move on.

                        ##### Checks whether the deviation between the windows "begin"
                        ##### and "end" times is less the tolerance value.
                        try:
                            s_mismatch = abs(start - self.sorted_timeline[s_name][index_s])
                            assert s_mismatch < self.mismatchTolerance
                            self.logger.info("Start_Reference: " + str(start) + " StartedAt: "
                                             + str(self.sorted_timeline[s_name][index_s])
                                             + " Tol: " + str(self.mismatchTolerance) + " Dev: "
                                             + str(s_mismatch) + " Passed: YES")
                        except Exception as e:
                            self.logger.warn("Start_Reference: " + str(start) + " StartedAt: "
                                             + str(self.sorted_timeline[s_name][index_s])
                                             + " Tol: " + str(self.mismatchTolerance) + " Dev: "
                                             + str(s_mismatch) + " Passed: NO")
                        try:
                            e_mismatch = abs(end -self.sorted_timeline[s_name][index_e])
                            assert e_mismatch < self.mismatchTolerance
                            self.logger.info("End_Reference: " + str(end) + " StartedAt: "
                                             + str(self.sorted_timeline[s_name][index_e]) + " Dev: "
                                             + " Tol: " + str(self.mismatchTolerance)
                                             + str(e_mismatch) + " Passed: YES" )
                        except Exception as e:
                            self.logger.warn("End_Reference: " + str(end) + " StartedAt: "
                                             + str(self.sorted_timeline[s_name][index_e]) + " Dev: "
                                             + " Tol: " + str(self.mismatchTolerance)
                                             + str(e_mismatch) + " Passed: NO" )

                        ##### Prints the windows content (row batch) to the corresponding (s_name) csv file.
                        self.csv_writers[s_name].writerows(buffer)  #write content to the file
                        self.csv_writers[s_name].writerows([{}])    #write an empty line to mark the end of the windows
                        self.output_filenames[s_name].flush()       #flush data.
                        print ''

        except:
            self.logger.error(traceback.format_exc())

    def setOutputCSVFiles(self):
        """Define file streams for saving the data. It generates as much files as those
        defined in the annotation ".json" source field. In other words, one for each tab
        (feature perspective) used for tagging the data."""

        # defaults the core name of the output file(s) to the name of the output_dir + bagName.
        defaultname = os.path.join(self.output_dir,self.current_bagName.split("/")[-1][:-4])

        try:
            # variable that holds the outputfiles for each perspective. Type: dictionary.
            self.output_filenames = {}
            #loop through perspectives.
            for s_name in self.annotationDictionary["sources"]:
                # append to the filename the feature perspective name and csv extension.
                filename = defaultname + "_" + s_name + ".csv"

                self.logger.info("Files generated: " + filename)
                # set output files
                self.output_filenames[s_name] = open(filename, 'wa')

                # define the headers for the csv files (annotated feature names + selected topics).
                self.csv_writers[s_name] = csv.DictWriter(self.output_filenames[s_name],
                                                          ["time"] + self.annotationDictionary[s_name]["labels"]
                                                          + self.topicSelectionONHeaders)
                # write the headers
                self.csv_writers[s_name].writeheader()
                # flush data
                self.output_filenames[s_name].flush()
        except:
            self.logger.critical(traceback.format_exc())
            sys.exit(-1)

        # loop through the data printing the windows content.
        self.writeData()

    def errorMessages(self,index):
        """Defines error messages via index parameter. Useful for avoiding
        replication of common error messages."""
        msg = None

        if index == 0:
            msg = "Error: It was not possible to load the bag file!" \
                           "Reason: topic incompatibility."
        elif index == 1:
            msg = "Error: It was not possible to load the annotation file!" \
                           "Reason: topic incompatibility."
        elif index == 2:
            msg = "Error: You must select the topics you are interested."
        elif index == 3:
            msg = "Error: You must load a bag file and/or an annotation file!"
        elif index == 4:
            msg = "Error: Error when opening the bag file!"
        elif index == 5:
            msg = "Error: Error when opening the annotation json file!"
        self.logger.error(msg)

    def getMatchedFiles(self):
        """Counts the files in the bag_dir and annotation dir. Return a list
        of files that match by name. In other words, a list of files that have
        the same name, but one being .json and other .bag."""
        #TODO : THIS METHOD MAY BE FRAGILE! CORRECT FOR POSSIBLE ERROR SITUATIONS.
        # remove extensions to enable filename comparison.
        bag_without_ext = [t[:-4] for t in self.allBagFiles if t.endswith(".bag")]
        ann_without_ext = [t[:-5] for t in self.allJsonFiles if t.endswith(".json")]
        # count file occurrences
        cnt = Counter(bag_without_ext + ann_without_ext)
        return [k for k in cnt if cnt[k] ==2]

    def run(self):
        """Run over all bag and json files. Writing the data to the corresponding file"""

        listOfIncompatible = []
        listOfFailedOpening = []

        # get the list of ".json" ".bag" pairs (theirs file
        # name should match, be the same).
        matched_files = self.getMatchedFiles()
        self.logger.info("Files found: " + json.dumps([i+".bag" for i in matched_files]+
                                     [i+".json" for i in matched_files],indent=4))

        # Run over the files what are matched (that is, have a version ".json" and a version ".bag")
        for dataFile in matched_files:
            # reconstruct file extentions.
            annotationFile = dataFile+".json"
            bagFile = dataFile +".bag"
            print ''

            self.logger.info("Exporting data...")
            # if openning of the annotation files is successful...
            if self.openAnnotationFile(os.path.join(self.annotation_dir,annotationFile)):
                #load the annotation data for the current annotation json file.
                self.loadAnnotationData()

                # open the bag file.
                if self.openBagFile(os.path.join(self.bags_dir, bagFile)):
                    # load bag data.
                    self.loadBagData()

                    self.logger.info("Performing compatibility test between: " + annotationFile + " and " + bagFile)
                    # verifies compatibility, that is whether the files have the topics matched.
                    if self.areFileCompatible():
                        self.logger.info("PASSED.")
                    else:
                        self.logger.error("FAILED!!! Files topics do not match!, skipping file pair.")
                        # skip to next file in case current files
                        # are not compatible.
                        listOfIncompatible.append(dataFile)
                        continue

                    # if files are compatible and the user has not load which topic
                    # to save via the parser_config.json... Ask the user to enter the topics
                    # to save manually.
                    if not self.isAutomaticTopicSelection:
                        # keep prompting selection to the user in case
                        # he did not select the topics to save.
                        while not self.hasTopicSelected:
                            self.getUserTopicSelection()
                            # print message in case he keeps not selecting the topics.
                            if not self.hasTopicSelected:
                                self.logger.warn("You must select at least one topic to save.")
                                print ''
                        # set up output csv files and write data to them.
                        self.setOutputCSVFiles()
                    else:
                        # write data directly in case the user has set up the parser_config.jason
                        self.setOutputCSVFiles()

                    # reset important variable, for allowing new
                    # ".json" and ".bag" to be loaded.
                    self.reset()
                else:
                    # skip to next file in case something go
                    # wrong while opening bag file.
                    self.logger.error("FAILED to open {} bag file! Skipping file pair.".format(dataFile))
                    listOfFailedOpening.append(dataFile)
                    continue
            else:
                # skip to next file in case something go
                # wrong withe opening annotation file.
                self.logger.error("FAILED to open {} annotation file! Skipping file pair.".format(dataFile))
                listOfFailedOpening.append(dataFile)
                continue

            print ''
            print ''
            self.logger.info("Program terminated with {} ({}) failed attempt(s) to open files and {} ({}) non-compatible "
                             ".bag/.json files pairs".format(len(listOfFailedOpening), str(listOfFailedOpening),
                                                             len(listOfIncompatible), str(listOfIncompatible)))