// carpet_detect_context.h
#pragma once
#include "yolov8_detect.h"
#include "transform_coordinates.h"
#include "track_filter.h"

struct DetectContext
{
    bool initialized = false;
    ConfigInfo config;
    Detector *detector = nullptr;
    CameraParameters *camera_params = nullptr;
    TrackFilter *detection_track_filter = nullptr;
};
