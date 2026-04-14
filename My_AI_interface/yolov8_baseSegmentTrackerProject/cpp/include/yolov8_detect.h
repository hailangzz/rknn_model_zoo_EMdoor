// Copyright (c) 2023 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef _RKNN_DEMO_YOLOV8_H_
#define _RKNN_DEMO_YOLOV8_H_
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <sstream>
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "types.h"
#include "postprocess.h"

ConfigInfo readConfig(const std::string& filename);

class Detector {

    public:
        Detector(const ConfigInfo& config);
        ~Detector();
        
        int inference_yolov8_model(image_buffer_t* img, object_detect_result_list* od_results);

    private:
        
        int init_yolov8_model(const char* model_path);
        int release_yolov8_model();

        void dump_tensor_attr(rknn_tensor_attr *attr);

        rknn_context ctx_ = 0;
        rknn_app_context_t rknn_app_ctx_;
        image_buffer_t src_image_;
        const std::string config_file_ = "./config/cfg.txt"; // 模型参数文件

};



#endif //_RKNN_DEMO_YOLOV8_H_