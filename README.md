# yolo_ros_vino

An implemntation of YOLO object detection with a ROS interface and enhanced processor utilization (40-20 fps) using OpenVINO model optimization tools. The work is based on [this](https://github.com/PINTO0309/OpenVINO-YoloV3) and [this](
https://github.com/leggedrobotics/darknet_ros).



# How to convert your trained darknet model to OpenVINO  intermediate representation (IR)
Clone the following packages:
- `git clone https://github.com/Haya-Alsharif/OpenVINO-YoloV3 ~/OpenVINO-YoloV3`
- `git clone https://github.com/Haya-Alsharif/yolo_ros_vino ~/catkin_ws/src/yolo_ros_vino/`

## from darknet to tensorflow (TF)
1) Create/check the labels to make sure they are in the approperiate order. Make sure that the end of the file has NO EMPTY LINES. More than one empty line will result in parsing the file incorrectly and will result later in compilation errors. The labels can be found at,
```
cd ~/OpenVINO-YoloV3/labels
gedit *.labels
```

2) Copy your weight files to `darknet_model` folder. Note that we wont use `.cfg`, but will use some it to modify the NN architecture. 

3) Convert from darknet to tensorflow NCHW or NHWC
```
cd ~/OpenVINO-YoloV3
python3 convert_weights_pb.py \
--class_names labels/yolov3_tiny_tags.labels \
--weights_file darknet_models/yolov3_tiny_tags.weights \
--output_graph tf_models/yolov3_tiny_tags.pb \
--data_format NHWC \
--size 416 \
--tiny
```

## from tensorflow (TF) to intermediate representation (IR)
1) Create/check the json files under `tf_models` directory. Sampels files were provided
```
gedit tf_models/yolov3_tiny_tags.json
```
The values in your json file are determend by your NN architecture described in `.cfg` file (right after the word `[yolo]` inside your `.cfg` file). 

2) Convert tensorflow to intermediate representation (IR) format

- for processing on your CPU
```
python3 ~/intel/computer_vision_sdk/deployment_tools/model_optimizer/mo_tf.py \
--input_model tf_models/yolov3_tiny_tags.pb \
--tensorflow_use_custom_operations_config tf_models/yolov3_tiny_tags.json \
--output_dir ir_models/tiny-YoloV3/FP32 \
--batch 1 \
--data_type FP32 \
--reverse_input_channels
```

- for processing on a Neural Compute Stick 2 (NCS2) (There is an issue with model conversion that is still not solved)
```
python3 ~/intel/computer_vision_sdk/deployment_tools/model_optimizer/mo_tf.py \
--input_model tf_models/yolov3_tiny_tags.pb \
--tensorflow_use_custom_operations_config tf_models/yolov3_tiny_tags.json \
--output_dir ir_models/tiny-YoloV3/FP16 \
--batch 1 \
--data_type FP16 \
--reverse_input_channels
```

3) Modify your model paramters, including the locaiton of the converted files in yolo_ros_vino.launch and enable or disable your `neural_compute_stick` variable 
```
gedit ~/catkin_ws/src/yolo_ros_vino/launch/yolo_ros_vino.launch
```

4) finally launch your camera and yolo_ros_vino packages
- `roslaunch realsense2_camera rs_camera.launch`
- `roslaunch yolo_ros_vino yolo_ros_vino.launch`
