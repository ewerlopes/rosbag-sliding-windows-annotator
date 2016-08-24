# Rosbag Annotator
This program will help you annotate a rosbag file by producing a result file with timestamps of the annotated events.
Currently works only for Image and CompressedImage topics 

It takes as input the rosbag file and a csv file if exists with annotated(or not) bound boxes from the video.
The csv file holds the annotations from the video and some other useful metrics.
It playbacks the rosbag file and give you some basic controls over the display and the ability to draw bound
boxes on the video surface which saves them in the csv when exiting the application.
