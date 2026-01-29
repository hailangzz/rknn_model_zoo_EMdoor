#include "yolov8_detect.h"

ConfigInfo readConfig(const std::string& filename) {
    ConfigInfo cfg_values;

    std::unordered_map<std::string, std::string> config;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << filename << std::endl;
        return cfg_values;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue; // 跳过空行和注释

        std::istringstream iss(line);
        std::string key, value;
        if (std::getline(iss, key, '=') && std::getline(iss, value)) {
            config[key] = value;
        }
    }

    cfg_values.model_path  = config["model_path"].c_str(); 
    cfg_values.input_width  = std::stoi(config["input_width"]);  
    cfg_values.input_height  = std::stoi(config["input_height"]);  
    
    cfg_values.score_threshold = std::stof(config["score_threshold"]); // 输入图片尺寸
    cfg_values.max_frame_threshold = std::stoi(config["max_frame_threshold"]);
    cfg_values.camera_z_axle_top_resize_rate = std::stof(config["camera_z_axle_top_resize_rate"]); // 相机坐标系，Z轴远端缩放比例
    cfg_values.camera_z_axle_polyfit_w0 = std::stof(config["camera_z_axle_polyfit_w0"]); 
    cfg_values.camera_z_axle_polyfit_w1 = std::stof(config["camera_z_axle_polyfit_w1"]); 
    cfg_values.camera_z_axle_polyfit_w2 = std::stof(config["camera_z_axle_polyfit_w2"]); 

    //相机参数配置：
    cfg_values.camera_fx = std::stof(config["camera_fx"]); 
    cfg_values.camera_fy = std::stof(config["camera_fy"]); 
    cfg_values.camera_cx = std::stof(config["camera_cx"]); 
    cfg_values.camera_cy = std::stof(config["camera_cy"]); 
    cfg_values.camera_H = std::stof(config["camera_H"]); 
    cfg_values.camera_pitch = std::stof(config["camera_pitch"]);
    cfg_values.camera_D_0 = std::stof(config["camera_D_0"]);
    cfg_values.camera_D_1 = std::stof(config["camera_D_1"]);
    cfg_values.camera_D_2 = std::stof(config["camera_D_2"]);
    cfg_values.camera_D_3 = std::stof(config["camera_D_3"]);
    cfg_values.camera_D_4 = std::stof(config["camera_D_4"]);
    cfg_values.camera_D_5 = std::stof(config["camera_D_5"]);
    cfg_values.camera_D_6 = std::stof(config["camera_D_6"]);
    cfg_values.camera_D_7 = std::stof(config["camera_D_7"]);

    return cfg_values;
}

// 析构函数释放 RKNN 资源
Detector::Detector(const ConfigInfo& config) {
    memset(&rknn_app_ctx_, 0, sizeof(rknn_app_context_t));

    init_yolov8_model(config.model_path.c_str());
}

// 析构函数释放 RKNN 资源
Detector::~Detector() {
    if (ctx_ != 0) {
        ctx_ = release_yolov8_model();
        if (ctx_ != 0)
        {
            printf("release_yolov8_model fail! ret=%d\n", ctx_);
        }
    }
    // if (src_image_.virt_addr != NULL)
    // {
    //     #if defined(RV1106_1103) 
    //             dma_buf_free(rknn_app_ctx_.img_dma_buf.size, &rknn_app_ctx_.img_dma_buf.dma_buf_fd, 
    //                     rknn_app_ctx_.img_dma_buf.dma_buf_virt_addr);
    //     #else
    //             free(src_image_.virt_addr);
    //     #endif
    // }
    
}

void Detector::dump_tensor_attr(rknn_tensor_attr *attr)
{
    printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
           attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

int Detector::init_yolov8_model(const char *model_path)
{
    int ret;
    int model_len = 0;
    char *model;
    // Load RKNN Model
    model_len = read_data_from_file(model_path, &model);
    if (model == NULL)
    {
        printf("load_model fail!\n");
        return -1;
    }

    ret = rknn_init(&ctx_, model, model_len, 0, NULL);
    free(model);
    if (ret < 0)
    {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
    }

    // Get Model Input Output Number
    rknn_input_output_num io_num;
    ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    // Get Model Input Info
    printf("input tensors:\n");
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        // dump_tensor_attr(&(input_attrs[i]));
    }

    // Get Model Output Info
    // printf("output tensors:\n");
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        // dump_tensor_attr(&(output_attrs[i]));  //输出推理结果
    }

    // Set to context
    rknn_app_ctx_.rknn_ctx = ctx_;

    // TODO
    if (output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && output_attrs[0].type == RKNN_TENSOR_INT8)
    {
        rknn_app_ctx_.is_quant = true;
    }
    else
    {
        rknn_app_ctx_.is_quant = false;
    }

    rknn_app_ctx_.io_num = io_num;
    rknn_app_ctx_.input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    memcpy(rknn_app_ctx_.input_attrs, input_attrs, io_num.n_input * sizeof(rknn_tensor_attr));
    rknn_app_ctx_.output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    memcpy(rknn_app_ctx_.output_attrs, output_attrs, io_num.n_output * sizeof(rknn_tensor_attr));

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        printf("model is NCHW input fmt\n");
        rknn_app_ctx_.model_channel = input_attrs[0].dims[1];
        rknn_app_ctx_.model_height = input_attrs[0].dims[2];
        rknn_app_ctx_.model_width = input_attrs[0].dims[3];
    }
    else
    {
        printf("model is NHWC input fmt\n");
        rknn_app_ctx_.model_height = input_attrs[0].dims[1];
        rknn_app_ctx_.model_width = input_attrs[0].dims[2];
        rknn_app_ctx_.model_channel = input_attrs[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n",
           rknn_app_ctx_.model_height, rknn_app_ctx_.model_width, rknn_app_ctx_.model_channel);

    return 0;
}

int Detector::release_yolov8_model()
{
    if (rknn_app_ctx_.input_attrs != NULL)
    {
        free(rknn_app_ctx_.input_attrs);
        rknn_app_ctx_.input_attrs = NULL;
    }
    if (rknn_app_ctx_.output_attrs != NULL)
    {
        free(rknn_app_ctx_.output_attrs);
        rknn_app_ctx_.output_attrs = NULL;
    }
    if (rknn_app_ctx_.rknn_ctx != 0)
    {
        rknn_destroy(rknn_app_ctx_.rknn_ctx);
        rknn_app_ctx_.rknn_ctx = 0;
    }
    return 0;
}

// int Detector::inference_yolov8_model(image_buffer_t *img, object_detect_result_list *od_results)
// {
//     int ret;
//     image_buffer_t dst_img;
//     letterbox_t letter_box;
//     rknn_input inputs[rknn_app_ctx_.io_num.n_input];
//     rknn_output outputs[rknn_app_ctx_.io_num.n_output];
//     const float nms_threshold = NMS_THRESH;      // 默认的NMS阈值
//     const float box_conf_threshold = BOX_THRESH; // 默认的置信度阈值
//     int bg_color = 114;

//     if ((!&rknn_app_ctx_) || !(img) || (!od_results))
//     {
//         return -1;
//     }

//     memset(od_results, 0x00, sizeof(*od_results));
//     memset(&letter_box, 0, sizeof(letterbox_t));
//     memset(&dst_img, 0, sizeof(image_buffer_t));
//     memset(inputs, 0, sizeof(inputs));
//     memset(outputs, 0, sizeof(outputs));

//     // Pre Process
//     dst_img.width = rknn_app_ctx_.model_width;
//     dst_img.height = rknn_app_ctx_.model_height;
//     dst_img.format = IMAGE_FORMAT_RGB888;
//     // dst_img.format = IMAGE_FORMAT_RGBA8888;
//     dst_img.size = get_image_size(&dst_img);
//     dst_img.virt_addr = (unsigned char *)malloc(dst_img.size);

//     if (dst_img.virt_addr == NULL)
//     {
//         printf("malloc buffer size:%d fail!\n", dst_img.size);
//         return -1;
//     }

//     // letterbox
//     ret = convert_image_with_letterbox(img, &dst_img, &letter_box, bg_color);
//     if (ret < 0)
//     {
//         printf("convert_image_with_letterbox fail! ret=%d\n", ret);
//         return -1;
//     }

//     // Set Input Data
//     inputs[0].index = 0;
//     inputs[0].type = RKNN_TENSOR_UINT8;
//     inputs[0].fmt = RKNN_TENSOR_NHWC;
//     inputs[0].size = rknn_app_ctx_.model_width * rknn_app_ctx_.model_height * rknn_app_ctx_.model_channel;
//     inputs[0].buf = dst_img.virt_addr;

//     ret = rknn_inputs_set(rknn_app_ctx_.rknn_ctx, rknn_app_ctx_.io_num.n_input, inputs);
//     if (ret < 0)
//     {
//         printf("rknn_input_set fail! ret=%d\n", ret);
//         return -1;
//     }

//     // Run
//     printf("rknn_run\n");
//     ret = rknn_run(rknn_app_ctx_.rknn_ctx, nullptr);
//     if (ret < 0)
//     {
//         printf("rknn_run fail! ret=%d\n", ret);
//         return -1;
//     }

//     // Get Output
//     memset(outputs, 0, sizeof(outputs));
//     for (int i = 0; i < rknn_app_ctx_.io_num.n_output; i++)
//     {
//         outputs[i].index = i;
//         outputs[i].want_float = (!rknn_app_ctx_.is_quant);
//     }
//     ret = rknn_outputs_get(rknn_app_ctx_.rknn_ctx, rknn_app_ctx_.io_num.n_output, outputs, NULL);
//     if (ret < 0)
//     {
//         printf("rknn_outputs_get fail! ret=%d\n", ret);
//         goto out;
//     }

//     // Post Process
//     post_process(&rknn_app_ctx_, outputs, &letter_box, box_conf_threshold, nms_threshold, od_results);

//     // Remeber to release rknn output
//     rknn_outputs_release(rknn_app_ctx_.rknn_ctx, rknn_app_ctx_.io_num.n_output, outputs);

// out:
//     if (dst_img.virt_addr != NULL)
//     {
//         free(dst_img.virt_addr);
//     }

//     return ret;
// }


int Detector::inference_yolov8_model(image_buffer_t *img, object_detect_result_list *od_results)
{
    if (!img || !od_results) {
        return -1;
    }

    int ret = -1;
    image_buffer_t dst_img;
    letterbox_t letter_box;
    rknn_input inputs[rknn_app_ctx_.io_num.n_input];
    rknn_output outputs[rknn_app_ctx_.io_num.n_output];
    const float nms_threshold = NMS_THRESH;      // NMS 阈值
    const float box_conf_threshold = BOX_THRESH; // 置信度阈值
    int bg_color = 114;

    memset(od_results, 0, sizeof(*od_results));
    memset(&letter_box, 0, sizeof(letterbox_t));
    memset(&dst_img, 0, sizeof(image_buffer_t));
    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    // -----------------------------
    // Step 1: 分配 dst buffer（对齐 + 初始化）
    // -----------------------------
    dst_img.width  = rknn_app_ctx_.model_width;
    dst_img.height = rknn_app_ctx_.model_height;
    dst_img.format = IMAGE_FORMAT_RGB888; // 确保 RGA/NPU 支持
    dst_img.size   = get_image_size(&dst_img);

    if (posix_memalign((void**)&dst_img.virt_addr, 64, dst_img.size) != 0) {
        printf("failed to allocate aligned memory for dst_img\n");
        return -1;
    }
    memset(dst_img.virt_addr, bg_color, dst_img.size); // 初始化背景

    // -----------------------------
    // Step 2: 检查输入格式
    // -----------------------------
    image_buffer_t src_img_aligned = *img;

    if (img->format == IMAGE_FORMAT_RGBA8888) {
        // 将 RGBA -> RGB（手动逐像素转换）
        unsigned char* src = img->virt_addr;
        unsigned char* dst = (unsigned char*)malloc(img->width * img->height * 3);
        if (!dst) {
            printf("malloc failed for RGB conversion\n");
            free(dst_img.virt_addr);
            return -1;
        }

        for (int i = 0; i < img->width * img->height; i++) {
            dst[i * 3 + 0] = src[i * 4 + 0]; // R
            dst[i * 3 + 1] = src[i * 4 + 1]; // G
            dst[i * 3 + 2] = src[i * 4 + 2]; // B
        }

        src_img_aligned.virt_addr = dst;
        src_img_aligned.format    = IMAGE_FORMAT_RGB888;
        src_img_aligned.size      = img->width * img->height * 3;
    }

    // -----------------------------
    // Step 3: letterbox 预处理
    // -----------------------------
    ret = convert_image_with_letterbox(&src_img_aligned, &dst_img, &letter_box, bg_color);


    if (ret < 0) {
        printf("convert_image_with_letterbox fail! ret=%d\n", ret);
        if (img->format == IMAGE_FORMAT_RGBA8888)
            free(src_img_aligned.virt_addr);
        free(dst_img.virt_addr);
        return -1;
    }

    // -----------------------------
    // Step 4: 设置 RKNN 输入
    // -----------------------------
    inputs[0].index = 0;
    inputs[0].type  = RKNN_TENSOR_UINT8;
    inputs[0].fmt   = RKNN_TENSOR_NHWC;
    inputs[0].size  = dst_img.width * dst_img.height * 3; // RGB888
    inputs[0].buf   = dst_img.virt_addr;

    ret = rknn_inputs_set(rknn_app_ctx_.rknn_ctx, rknn_app_ctx_.io_num.n_input, inputs);
    if (ret < 0) {
        printf("rknn_inputs_set fail! ret=%d\n", ret);
        goto out;
    }

    // -----------------------------
    // Step 5: 推理
    // -----------------------------
    ret = rknn_run(rknn_app_ctx_.rknn_ctx, nullptr);
    if (ret < 0) {
        printf("rknn_run fail! ret=%d\n", ret);
        goto out;
    }

    // -----------------------------
    // Step 6: 获取输出
    // -----------------------------
    for (int i = 0; i < rknn_app_ctx_.io_num.n_output; i++) {
        outputs[i].index      = i;
        outputs[i].want_float = (!rknn_app_ctx_.is_quant);
    }

    ret = rknn_outputs_get(rknn_app_ctx_.rknn_ctx, rknn_app_ctx_.io_num.n_output, outputs, NULL);
    if (ret < 0) {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        goto out;
    }

    // -----------------------------
    // Step 7: 后处理
    // -----------------------------
    post_process(&rknn_app_ctx_, outputs, &letter_box, box_conf_threshold, nms_threshold, od_results);

    rknn_outputs_release(rknn_app_ctx_.rknn_ctx, rknn_app_ctx_.io_num.n_output, outputs);

out:
    if (img->format == IMAGE_FORMAT_RGBA8888)
        free(src_img_aligned.virt_addr);

    if (dst_img.virt_addr)
        free(dst_img.virt_addr);

    return ret;
}
