/*
 * YoloRosVino.h
 *
 *  Created on: Feb 1, 2019
 *      Author: Haya Alsharif
 *   Institute: KAUST
 */

#pragma once

//C++
#include <math.h>
#include <functional>
#include <iostream>
#include <fstream>
#include <random>
#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>

//ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <yolo_ros_vino/BoundingBoxes.h>
#include <yolo_ros_vino/BoundingBox.h>

//OpenCv
#include <samples/ocv_common.hpp>
#include <samples/slog.hpp>
#include <cv_bridge/cv_bridge.h>

//xServer
#include <X11/Xlib.h>

//OpenVINO
#include <inference_engine.hpp>

#include <ext_list.hpp>


class YoloRosVino{
    public:

        explicit YoloRosVino(ros::NodeHandle nh);

        ~YoloRosVino();

        struct DetectionObject {
            int xmin, ymin, xmax, ymax, class_id;
            float confidence;
            std::string Class;

            DetectionObject(double x, double y, double h, double w, int class_id, std::string Class, float confidence, float h_scale, float w_scale);

            bool operator<(const DetectionObject &s2) const;

            yolo_ros_vino::BoundingBox BoundingBox();
        };

        bool ReadParameters();

        static int EntryIndex(int side, int lcoords, int lclasses, int location, int entry);

        double IntersectionOverUnion(const DetectionObject &box_1, const DetectionObject &box_2);

        void ParseYOLOV3Output(const InferenceEngine::CNNLayerPtr &layer, const InferenceEngine::Blob::Ptr &blob, const unsigned long resized_im_h, const unsigned long resized_im_w, const unsigned long original_im_h, const unsigned long original_im_w, const float threshold, std::vector<DetectionObject> &objects);

        void callback(const sensor_msgs::ImageConstPtr& current_image);


        ros::NodeHandle nodeHandle_;
        ros::Publisher boundingBoxesPublisher_;
        image_transport::Subscriber imageSubscriber_;
        image_transport::ImageTransport imageTransport_;

        std::string inputName_;
        std::string modelFileName_, binFileName_, labelFileName_;
        std::string cameraTopicName_;
        std::vector<std::string> labels_;

        float thresh_, iouThresh_;
        bool viewResult_, neuralComputeStick_;

        InferenceEngine::InferRequest::Ptr async_infer_request_curr_;
        InferenceEngine::OutputsDataMap outputInfo_;
        InferenceEngine::InputsDataMap inputInfo_;
        InferenceEngine::CNNNetReader netReader_;

        typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;

};





