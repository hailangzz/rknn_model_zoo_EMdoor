// carpet_detect_context.h
#pragma once
#include "yolov8_detect.h"
#include "transform_coordinates.h"

struct DetectContext {
    bool initialized = false;
    ConfigInfo config;
    Detector* detector = nullptr;
    CameraParameters* camera_params = nullptr;
};


