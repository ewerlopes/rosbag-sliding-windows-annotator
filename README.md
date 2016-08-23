# rosbag_annotator_v2
This program will help you annotate a rosbag file by producing a result file with timestamps of the annotated events.
Currently works only for Image and CompressedImage topics 

It takes as input the rosbag file from the GUI and the csv file which holds the annotations from the
video and some other useful metrics.
It playbacks the rosbag file and give you some basic controls over the display.
It allows you to pause, rewind and annotate on the video using bound boxes on top of it and then saves them
in the csv file by exiting the programm.

 

