# Rosbag Sliding Window Annotator
![Language](https://img.shields.io/badge/Python-2.7-blue.svg) ![ROS](https://img.shields.io/badge/ROS-Kinetic%20Kame-brightgreen.svg) ![Ubuntu](https://img.shields.io/badge/Ubuntu-16.04LTS-orange.svg)
![Version](https://img.shields.io/badge/version-1.0-brightgreen.svg) ![License MIT](https://img.shields.io/cocoapods/l/AFNetworking.svg)

This project aims at providing a way to annotate rosbag files by using the method of sliding windows. The annotation procedure is done by using the `annotator.py` script. At the end of the annotation procedure, the script generates a json file with timestamps and other general information about the data tagged. The generated json file, together with its associated bag file is then feed into a second script called the `annotation_parser.py` that extract the actual data from the rosbag into a `csv` file, for further processing.

**Note that the objective of this program is to use a video image topic as a guide for tagging numerical data**. It is useful, for instance, if you want to work with activity recognition and need to annotate data from numerical sensors, like accelerometers, that were recorded in a given moment in time. In this case, the image topic is used as a guide for tagging numerical data in the same rosbag file, since using only the numerical values would be hard.

**Currently allows only ros image topics of type `compressedImage` to be loaded.**

## DEPENDENCIES
* **OS:** `Ubuntu 16.04LTS`
* **Python:** `Version 2.7.xx`. You can check your version from terminal, with: `python -V`.


* **Ros:** [`Kinetic Kame`](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* **Libraries:**

```bash
$ sudo apt-get install ros-kinetic-python-qt-binding pyqt5-dev pyqt5-dev-tools python-pyqt5.qtmultimedia python-pyqt5
```
##### Possible compementary dependencies:
Depending on how you install ros (desktop-full or just desktop), you may run into the `defaultServiceProvider::requestService(): no service found for - "org.qt-project.qt.mediaplayer"` problem when playing the rosbag image topics in the `annotator.py` script. It is a problem most likely related with the qtmultimedia part of the pyqt5 library. A quick solution would be to install qtmultimedia5.examples, by issuing the folowing command in the terminal:

```sudo apt-get install qtmultimedia5-examples```

## Setup and Usage

Before using the scripts, place in the `config.json` file, that should be placed in the same directory location of the other scripts, the set of label, and their values, that are going to be considered during the annotation. **The current version only allows for mutually exclusive labels, though**. Below, there is an example of a valid `config.json`:

```json
{
    "Human": {
                "Activity": ["Walking", "Running", "Jumping"],
                "Speed": ["Slow", "Normal", "Fast", "Very Fast"],
                "Force Intensity": ["Weak", "Medium", "Strong"],
              },

    "Robot": {
    			  "Speed": ["Slow", "Normal", "fast"],
    			  "Is Matching Human?" : ["No","Almost","Yes"]
    }
}
```

Note that if must basically follow the python dictinary sintax. At this point in version, only two nested values are allowed in the config.jason. That is, a more broad feature perspective ("Human", "Robot") and the feature labels themselves with their values being a list of strings (In case of just one value, place it as a single-element list). The called feature perspectives are used for grouping the labels into tabs in the `annotator.py` interface. This is directed for the case where the annotating data that have multiple tags perspective, for instance, we can annotate the data with the human perspective or doing that taking into consideration the robot behavior in the scene or both.

Run the annotator with `annotator.py` command for actual data annotation. You can control parameters like: `overlap`: the amount of overlap between consecutive windows; `windows size`: the size of the data windows in seconds. Note that you should make sure you are using the right image topic for the selection. A image topic selection combo box is present in the interface.

**At the moment, the annotation_parser can be run in two different version. The GUI version allows you to generate csv files one at time. The consoled version is used when you want to generate the csv files from a collection of `.bag` annotation `.jason` files.**

Runnning the `annotation_parser.py --gui` if you are interested in getting the annotated bag file data from the corresponding generated json file and its associated rosbag file. Using the GUI version, load the rosbag file and its associated annotated json file using the appropriated buttons, choose the topics you want to extract and press the `Export CSV` button. The program then is going to save csv files with the bag data, given the annotation described in the json file. It generates one csv file for each perspective, taking into account ther corresponding annotations in the jason.

Running `annotation_parser.py` you run the console version. In this case the current file version receives as parameters the folder where to locate the bag files; the folder where to locate the annotation json files and a folder where to save the csv. In case no parameter is give, the mentioned directories are defaulted to the directory where the script is located. After identifying the files in the given directories the program loops **over ALL at once** and writes their data to their appropriated csv files and location. There are two basic methods from which to inform which data to save:

1. by setting up a python dictionary structure where topic atribute is a key and "ON" or "OFF" are the values, indicating whether you want to save that value. Ommiting a topic attribute on the dictionary is going to make the program ignore it when saving the data. The dictionary should be saved in a file called `parser_config.json`, located in script folder.
2. Not setting the `parser_config.json` makes the program prompt the user for selecting topics to save each time it loads a rosbag and annotation file pair.

You can also set the mismatch tolerance value for the consoled version. Execute `annotation_parser.py --help` in order to see the available program parameters.

## Debugging
The current version also includes debugging script. In the folder `debug` you can use the `debug.py` file for printing the summary statistics for the generated csv data files. Use `./debug.py --help` in the terminal in order to know more.

Get involved!
-------------

I am happy to receive bug reports, fixes, documentation enhancements, and other improvements.

Please report bugs via the `github issue` tracker.

Master git repository:

`https://github.com/ewerlopes/rosbag-sliding-windows-annotator.git`

If you want to contribute, here is a TODO list of what would be interesting to do:

#### TODO
1. prevent program crash if the config.json is not correctly formated.
2. combine `the annotation_parser.py` into the `annotator.py`. In case we want to generate the CSV directly after completing the annotation in the `annotator.py`.
3. add multithreading for avoiding getting an unresponsable windows if a function call (like opening a file) takes time to be completed.
4. allowing for plotting numerical values of selected topics when annotating. This would be useful if we need to make sure the numeral values are behaving in a desired way.
5. allowing to ignore tagging a certain windows.
6. remove CompressedImage type restriction to the image topics.
7. making the parser_GUI.py process a collection of .bag/.json files at once.

LICENSE
-------
The MIT License (MIT)

Copyright (c) 2016 Ewerton Lopes

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Authors
-------
These scripts are maintained by Ewerton Lopes and Davide Orrù. Parts of the GUI interface code were originally designed by @Diminal, at this [fork](https://github.com/dimimal/rosbag_annotator). However, his work is a folow up of the work of @dsou in [here](https://github.com/dsgou/rosbag_annotator.git), both developers focusing on the task of annotating video. This project, however, focus on the idea of annotating data (numeral values), with the help of an image data.
