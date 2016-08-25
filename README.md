# Rosbag Annotator
This program will help you annotate a rosbag file by producing a result file with timestamps of the annotated events.
Currently works only for Image and CompressedImage topics 

It takes as input the rosbag file and a csv file if exists with annotated(or not) bound boxes from the video.
The csv file holds the annotations from the video and some other useful metrics.
It playbacks the rosbag file and give you some basic controls over the display and the ability to draw bound
boxes on the video surface which saves them in the csv when exiting the application.

####This video player annotator is based on pyqt5 library.

## DEPENDENCIES
* **OS:** Ubuntu 16.04
* **Python 2.7.XX**

    You can check your version with: `python -V` command


* **Ros-Kinetic:** http://wiki.ros.org/kinetic/Installation/Ubuntu
* Download synaptic package manager '''sudo apt-get install''' synaptic and select the libraries below:

      1. ros-kinetic-python-qt-binding

      2. pyqt5-dev

      3. pyqt5-dev-tools
      
      4. python-pyqt5.qtmultimedia

      5. python-pyqt5

* **MATPLOTLIB**

      `sudo apt-get install python-matplotlib`
  
* **NUMPY**

      `sudo apt-get install python-numpy`

## Execution

Run the annotator with `./rosbag_annotator_v2.py` command.

_labels.json_ file is used to add or remove any class label for annotation usage.

