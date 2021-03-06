# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Byte
import threading
import time
import json
import pickle
from collections.abc import ByteString
import numpy as np
import cv2
import tensorflow as tf
from object_detection.utils import label_map_util
from object_detection.utils import config_util
from object_detection.utils import visualization_utils as viz_utils
from object_detection.builders import model_builder
from scipy.optimize import linear_sum_assignment
import base64
from py_pubsub import sort



ros_coordinates = []

ros_tracks_detections =  [] 
    
def numpy_to_bytes(arr: np.array) -> str:
    arr_dtype = bytearray(str(arr.dtype), 'utf-8')
    arr_shape = bytearray(','.join([str(a) for a in arr.shape]), 'utf-8')
    sep = bytearray('|', 'utf-8')
    arr_bytes = arr.ravel().tobytes()
    to_return = arr_dtype + sep + arr_shape + sep + arr_bytes
    return to_return

def bytes_to_numpy(serialized_arr: str) -> np.array:
    sep = '|'.encode('utf-8')
    i_0 = serialized_arr.find(sep)
    i_1 = serialized_arr.find(sep, i_0 + 1)
    arr_dtype = serialized_arr[:i_0].decode('utf-8')
    arr_shape = tuple([int(a) for a in serialized_arr[i_0 + 1:i_1].decode('utf-8').split(',')])
    arr_str = serialized_arr[i_1 + 1:]
    arr = np.frombuffer(arr_str, dtype = arr_dtype).reshape(arr_shape)
    return arr    


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.tracker = sort.Sort(max_age=5, min_hits=1)
        self.detection_model = None
        self.cost_threshold = 15
        
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.4  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        x = threading.Thread(target=self.objects_detector, args=(1,))
        x.start()

    @tf.function
    def detect_fn(self, image):
        """Detect objects in image."""
    
        image, shapes = self.detection_model.preprocess(image)
        prediction_dict = self.detection_model.predict(image, shapes)
        detections = self.detection_model.postprocess(prediction_dict, shapes)
    
        return detections, prediction_dict, tf.reshape(shapes, [-1])

    def objects_detector(self, name):

        global ros_coordinates
        """
        Detect Objects Using Your Webcam
        ================================
        """
        
        # %%
        # This demo will take you through the steps of running an "out-of-the-box" detection model to
        # detect objects in the video stream extracted from your camera.
        
        # %%
        # Create the data directory
        # ~~~~~~~~~~~~~~~~~~~~~~~~~
        # The snippet shown below will create the ``data`` directory where all our data will be stored. The
        # code will create a directory structure as shown bellow:
        #
        # .. code-block:: bash
        #
        #     data
        #     ????????? models
        #
        # where the ``models`` folder will will contain the downloaded models.
        import os
        
        DATA_DIR = os.path.join(os.getcwd(), 'data')
        MODELS_DIR = os.path.join(DATA_DIR, 'models')
        for dir in [DATA_DIR, MODELS_DIR]:
            if not os.path.exists(dir):
                os.mkdir(dir)
        
        # %%
        # Download the model
        # ~~~~~~~~~~~~~~~~~~
        # The code snippet shown below is used to download the object detection model checkpoint file,
        # as well as the labels file (.pbtxt) which contains a list of strings used to add the correct
        # label to each detection (e.g. person).
        #
        # The particular detection algorithm we will use is the `SSD ResNet101 V1 FPN 640x640`. More
        # models can be found in the `TensorFlow 2 Detection Model Zoo <https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md>`_.
        # To use a different model you will need the URL name of the specific model. This can be done as
        # follows:
        #
        # 1. Right click on the `Model name` of the model you would like to use;
        # 2. Click on `Copy link address` to copy the download link of the model;
        # 3. Paste the link in a text editor of your choice. You should observe a link similar to ``download.tensorflow.org/models/object_detection/tf2/YYYYYYYY/XXXXXXXXX.tar.gz``;
        # 4. Copy the ``XXXXXXXXX`` part of the link and use it to replace the value of the ``MODEL_NAME`` variable in the code shown below;
        # 5. Copy the ``YYYYYYYY`` part of the link and use it to replace the value of the ``MODEL_DATE`` variable in the code shown below.
        #
        # For example, the download link for the model used below is: ``download.tensorflow.org/models/object_detection/tf2/20200711/ssd_resnet101_v1_fpn_640x640_coco17_tpu-8.tar.gz``
        import tarfile
        import urllib.request
        
        # Download and extract model
        MODEL_DATE = '20200711'
        MODEL_NAME = 'ssd_resnet101_v1_fpn_640x640_coco17_tpu-8'
        MODEL_TAR_FILENAME = MODEL_NAME + '.tar.gz'
        MODELS_DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/tf2/'
        MODEL_DOWNLOAD_LINK = MODELS_DOWNLOAD_BASE + MODEL_DATE + '/' + MODEL_TAR_FILENAME
        PATH_TO_MODEL_TAR = os.path.join(MODELS_DIR, MODEL_TAR_FILENAME)
        PATH_TO_CKPT = os.path.join(MODELS_DIR, os.path.join(MODEL_NAME, 'checkpoint/'))
        PATH_TO_CFG = os.path.join(MODELS_DIR, os.path.join(MODEL_NAME, 'pipeline.config'))
        if not os.path.exists(PATH_TO_CKPT):
            print('Downloading model. This may take a while... ', end='')
            urllib.request.urlretrieve(MODEL_DOWNLOAD_LINK, PATH_TO_MODEL_TAR)
            tar_file = tarfile.open(PATH_TO_MODEL_TAR)
            tar_file.extractall(MODELS_DIR)
            tar_file.close()
            os.remove(PATH_TO_MODEL_TAR)
            print('Done')
        
        # Download labels file
        LABEL_FILENAME = 'mscoco_label_map.pbtxt'
        LABELS_DOWNLOAD_BASE = \
            'https://raw.githubusercontent.com/tensorflow/models/master/research/object_detection/data/'
        PATH_TO_LABELS = os.path.join(MODELS_DIR, os.path.join(MODEL_NAME, LABEL_FILENAME))
        if not os.path.exists(PATH_TO_LABELS):
            print('Downloading label file... ', end='')
            urllib.request.urlretrieve(LABELS_DOWNLOAD_BASE + LABEL_FILENAME, PATH_TO_LABELS)
            print('Done')
        
        # %%
        # Load the model
        # ~~~~~~~~~~~~~~
        # Next we load the downloaded model
        
        os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'    # Suppress TensorFlow logging

        
        tf.get_logger().setLevel('ERROR')           # Suppress TensorFlow logging (2)
        
        # Enable GPU dynamic memory allocation
        gpus = tf.config.experimental.list_physical_devices('GPU')
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        
        # Load pipeline config and build a detection model
        configs = config_util.get_configs_from_pipeline_file(PATH_TO_CFG)
        model_config = configs['model']
        self.detection_model = model_builder.build(model_config=model_config, is_training=False)
        
        # Restore checkpoint
        ckpt = tf.compat.v2.train.Checkpoint(model=self.detection_model)
        ckpt.restore(os.path.join(PATH_TO_CKPT, 'ckpt-0')).expect_partial()
        

        
        
        # %%
        # Load label map data (for plotting)
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Label maps correspond index numbers to category names, so that when our convolution network
        # predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility
        # functions, but anything that returns a dictionary mapping integers to appropriate string labels
        # would be fine.
        category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS,
                                                                            use_display_name=True)
        
        # %%
        # Define the video stream
        # ~~~~~~~~~~~~~~~~~~~~~~~
        # We will use `OpenCV <https://pypi.org/project/opencv-python/>`_ to capture the video stream
        # generated by our webcam. For more information you can refer to the `OpenCV-Python Tutorials <https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_video_display/py_video_display.html#capture-video-from-camera>`_
        #import cv2
        
        cap = cv2.VideoCapture(0)
        
        # %%
        # Putting everything together
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # The code shown below loads an image, runs it through the detection model and visualizes the
        # detection results, including the keypoints.
        #
        # Note that this will take a long time (several minutes) the first time you run this code due to
        # tf.function's trace-compilation --- on subsequent runs (e.g. on new images), things will be
        # faster.
        #
        # Here are some simple things to try out if you are curious:
        #
        # * Modify some of the input images and see if detection still works. Some simple things to try out here (just uncomment the relevant portions of code) include flipping the image horizontally, or converting to grayscale (note that we still expect the input image to have 3 channels).
        # * Print out `detections['detection_boxes']` and try to match the box locations to the boxes in the image.  Notice that coordinates are given in normalized form (i.e., in the interval [0, 1]).
        # * Set ``min_score_thresh`` to other values (between 0 and 1) to allow more detections in or to filter out more detections.
        #import numpy as np
        
        while True:
            # Read frame from camera
            ret, image_np = cap.read()
        
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image_np, axis=0)
        
            # Things to try:
            # Flip horizontally
            # image_np = np.fliplr(image_np).copy()
        
            # Convert image to grayscale
            # image_np = np.tile(
            #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)
        
            input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.float32)
            detections, predictions_dict, shapes = self.detect_fn(input_tensor)
            #print("1111111111111")
            # print(detections['detection_boxes'][0].numpy())
            # print("22222222222222")
            #print(detections['detection_scores'][0].numpy())
            #print(predictions_dict)
        
            label_id_offset = 1
            image_np_with_detections = image_np.copy()
        
            viz_utils.visualize_boxes_and_labels_on_image_array(
                  image_np_with_detections,
                  detections['detection_boxes'][0].numpy(),
                  (detections['detection_classes'][0].numpy() + label_id_offset).astype(int),
                  detections['detection_scores'][0].numpy(),
                  category_index,
                  use_normalized_coordinates=True,
                  max_boxes_to_draw=200,
                  min_score_thresh=.30,
                  agnostic_mode=False)
            
            #print(len(image_np_with_detections))
            #print(type(image_np_with_detections))
            
        
            # This is the way I'm getting my coordinates
            boxes = detections['detection_boxes'].numpy()[0]
            # get all boxes from an array
            max_boxes_to_draw = boxes.shape[0]
            # get scores to get a threshold
            scores = detections['detection_scores'].numpy()[0]
            # this is set as a default but feel free to adjust it to your needs
            min_score_thresh=.3
            # # iterate over all objects found
            coordinates = []
            
            tracks_detections = []
            for i in range(min(max_boxes_to_draw, boxes.shape[0])):
                if scores[i] > min_score_thresh:
                    class_id = int(detections['detection_classes'].numpy()[0][i] + 1)
                    coordinates.append({
                        "box": boxes[i],
                        "class_name": category_index[class_id]["name"],
                        "score": scores[i]
                    })
                    if category_index[class_id]["name"] == "person":
                        det_list = np.array([[boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3], 0.01]])
                        x =  boxes[i][0] * 2336
                        y = boxes[i][1] * 1080
                        width = boxes[i][2] * 2336  - x
                        height = boxes[i][3] * 1080 - y
                        score = scores[i]
                        det_list = np.vstack((det_list, \
                            [x, y, x+width, y+height, score]))
                        print("1111111")
                        print(det_list)                                 
                        print("1111111")
                        tracks_detections.append(det_list)
            
            
            #print(coordinates)
            #ros_coordinates = coordinates
            #print(len(ros_coordinates))
            #print()   
        
            # Display output
            cv2.imshow('object detection', cv2.resize(image_np_with_detections, (1024, 472)))
    
    
            """
            det_list = np.array([[0, 0, 1, 1, 0.01]])
    
            if len(detections) > 0:
                for i, detection in enumerate(detections):
    
                    if True:
                        x =  detection.mask.roi.x
                        y = detection.mask.roi.y
                        width = detection.mask.roi.width
                        height = detection.mask.roi.height
                        score = detection.score
    
                        det_list = np.vstack((det_list, \
                            [x, y, x+width, y+height, score]))
                    else:
                        del detections[i]
                        
            """
            
            data = []
            
            for det_list in tracks_detections:
                # Call the tracker
                tracks = self.tracker.update(det_list)
        
                # Copy the detections
                detections_copy = tracks_detections
        
                #detections = []
                
        
                if len(det_list) > 0:
        
                    # Create cost matrix
                    # Double for in Python :(
                    C = np.zeros((len(tracks), len(det_list)))
                    for i, track in enumerate(tracks):
                        for j, det in enumerate(det_list):
                            C[i, j] = np.linalg.norm(det[0:-2] - track[0:-2])
        
                    # apply linear assignment
                    row_ind, col_ind = linear_sum_assignment(C)
        
                    for i, j in zip(row_ind, col_ind):
                        if C[i, j] < self.cost_threshold and j != 0:
                            print("{} -> {} with cost {}".\
                                format(tracks[i, 4], detections_copy[j-1],\
                                C[i,j]))
        
                            #detections_copy[j-1].id = int(tracks[i, 4])
        
                            #detections.append(detections_copy[j-1])
                            
                            """
                            >>> aa = np.array([1.220,33.33,444.222])
                            >>> aa
                            array([  1.22 ,  33.33 , 444.222])
                            >>> base64.b64encode(aa)
                            b'hetRuB6F8z8K16NwPapAQGQ730+Nw3tA'
                            >>> cc = base64.b64encode(aa)
                            >>> dd = cc.decode("utf-8")
                            >>> dd
                            'hetRuB6F8z8K16NwPapAQGQ730+Nw3tA'
                            >>> ff = dd.encode("utf-8")
                            >>> xx = base64.b64decode(ff)
                            >>> 
                            >>> xx
                            b'\x85\xebQ\xb8\x1e\x85\xf3?\n\xd7\xa3p=\xaa@@d;\xdfO\x8d\xc3{@'
                            >>> np.frombuffer(xx, dtype=np.float64)
                            array([  1.22 ,  33.33 , 444.222])
                            >>> 
                            """
                            
                            bbb = base64.b64encode(np.around(detections_copy[j-1][0],4)).decode("utf-8")
                            ccc = base64.b64encode(np.around(detections_copy[j-1][1],4)).decode("utf-8")
                            
                            d = {
                                "id": "{}".format(tracks[i, 4]),
                                "coordinates": bbb,
                                "tracks_coordinates": ccc,
                                "cost": "{}".format(C[i,j])
                            }

                            
                            
                            data.append(d)
        
                    print("------------")
        
        
                else:
                    print("No tracked objects!")
                    
            
            ros_coordinates = data
            #self.pub_trackers.publish(detections)
    
    
    
    
    
        
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()        

    def timer_callback(self):
        global ros_coordinates
        msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #print(ros_coordinates)
        print(len(ros_coordinates))
        if ros_coordinates:
            d = json.dumps(ros_coordinates)
            msg.data = d
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1


    def timer_callback1(self):
        global ros_coordinates
        msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #print(ros_coordinates)
        print(len(ros_coordinates))
        res = []
        if ros_coordinates:
            for coordinate in ros_coordinates:
                box = coordinate["box"]
                class_name = coordinate["class_name"]
                score = coordinate["score"]
                if class_name == "person":
                    print("find person %s" % coordinate)
                    d = np.array2string(box, precision=5, separator=',',suppress_small=True)
                    print(d)
                    #b = pickle.dumps(ros_coordinates)
                    msg.data = d
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publishing: "%s"' % msg.data)
                    self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
