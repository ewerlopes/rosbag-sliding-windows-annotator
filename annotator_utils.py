from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import yaml, csv, os, cv2

def buffer_data(bag, input_topic, compressed):
    image_buff = []
    time_buff  = []
    start_time = None
    bridge     = CvBridge()

    #Buffer the images, timestamps from the rosbag
    for topic, msg, t in bag.read_messages(topics=[input_topic]):
        if start_time is None:
            start_time = t

        #Get the image
        if not compressed:
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print e
        else:
            nparr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(nparr, 1) #### cv2.CV_LOAD_IMAGE_COLOR has as enum value 1.
            ## TODO: fix the problem with the this enum value.

        image_buff.append(cv_image)
        time_buff.append(t.to_sec() - start_time.to_sec())

    return image_buff,  time_buff

#Returns a buffer with boxes
def buffer_csv(csv_file):
    box_buff   = []
    metrics = []
    box_buff_action = []

    if csv_file is not None and os.path.exists(csv_file):
        with open(csv_file, 'r') as file_obj:
            csv_reader = csv.reader(file_obj, delimiter = '\t')
            row_1 = next(csv_reader)
            try:
                index = [x.strip() for x in row_1].index('Rect_id')
                if 'Class' not in row_1:
                    for row in csv_reader:
                        (rec_id,x, y, width, height) = map(int, row[index:index + 5])
                        (meter_X,meter_Y,meter_Z,top,meter_h,distance) = map(float, row[(index+5)::])
                        box_buff.append((rec_id,x, y, width, height))
                        metrics.append((meter_X,meter_Y,meter_Z,top,meter_h,distance))
                else:
                    #index = [x.strip() for x in row_1].index('Rect_id')
                    for row in csv_reader:
                        (rec_id,x, y, width, height) = map(int, row[index:index + 5])
                        #action = map(str,row[index+5])
                        (meter_X,meter_Y,meter_Z,top,meter_h,distance) = map(float, row[(index+6)::])
                        box_buff.append((rec_id,x, y, width, height))
                        box_buff_action.append(str(row[index+5]))
                        metrics.append((meter_X,meter_Y,meter_Z,top,meter_h,distance))
            except:
                return False,False,False
            return box_buff,metrics,box_buff_action
    else:
        return False,False,False

def get_bag_metadata(bag):
    info_dict       = yaml.load(bag._get_yaml_info())
    topics             = info_dict['topics']
    topic            = topics[0]
    duration       = info_dict['duration']
    topic_type       = topic['type']
    message_count = topic['messages']

    #Messages for test
    print "\nRosbag topics found: "
    for top in topics:
        print "\t- ", top["topic"], "\n\t\t-Type: ", top["type"],"\n\t\t-Fps: ", top["frequency"]

    print 23 * '#'
    print topic_type
    #Checking if the topic is compressed
    if 'CompressedImage' in topic_type:
        compressed = True
    else:
        compressed = False

    #Get framerate
    framerate = message_count/duration

    return message_count,duration,compressed, framerate