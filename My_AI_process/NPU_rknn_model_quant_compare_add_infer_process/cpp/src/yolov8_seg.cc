#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "yolov8_seg.h"
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"

static void dump_tensor_attr(rknn_tensor_attr *attr)
{
    printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
           attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

int init_yolov8_seg_model(const char *model_path, rknn_app_context_t *app_ctx)
{

    int ret;
    int model_len = 0;
    char *model;
    rknn_context ctx = 0;

    // Load RKNN Model
    model_len = read_data_from_file(model_path, &model);
    if (model == NULL)
    {
        printf("load_model fail!\n");
        return -1;
    }

    ret = rknn_init(&ctx, model, model_len, 0, NULL);
    free(model);
    if (ret < 0)
    {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
    }

    // Get Model Input Output Number
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
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
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    // Get Model Output Info
    printf("output tensors:\n");
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(output_attrs[i]));
    }

    // Set to context
    app_ctx->rknn_ctx = ctx;

    // TODO
    if (output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && output_attrs[0].type != RKNN_TENSOR_FLOAT16)
    {
        app_ctx->is_quant = true;
    }
    else
    {
        app_ctx->is_quant = false;
    }

    app_ctx->io_num = io_num;
    app_ctx->input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->input_attrs, input_attrs, io_num.n_input * sizeof(rknn_tensor_attr));
    app_ctx->output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->output_attrs, output_attrs, io_num.n_output * sizeof(rknn_tensor_attr));

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        printf("model is NCHW input fmt\n");
        app_ctx->model_channel = input_attrs[0].dims[1];
        app_ctx->model_height = input_attrs[0].dims[2];
        app_ctx->model_width = input_attrs[0].dims[3];
    }
    else
    {
        printf("model is NHWC input fmt\n");
        app_ctx->model_height = input_attrs[0].dims[1];
        app_ctx->model_width = input_attrs[0].dims[2];
        app_ctx->model_channel = input_attrs[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n",
           app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);

    return 0;
}

int release_yolov8_seg_model(rknn_app_context_t *app_ctx)
{
    if (app_ctx->input_attrs != NULL)
    {
        free(app_ctx->input_attrs);
        app_ctx->input_attrs = NULL;
    }
    if (app_ctx->output_attrs != NULL)
    {
        free(app_ctx->output_attrs);
        app_ctx->output_attrs = NULL;
    }
    if (app_ctx->rknn_ctx != 0)
    {
        rknn_destroy(app_ctx->rknn_ctx);
        app_ctx->rknn_ctx = 0;
    }
    return 0;
}

int inference_yolov8_seg_model(rknn_app_context_t *rknn_app_ctx_, image_buffer_t *img, object_detect_result_list *od_results)
{
    if (!img || !od_results)
    {
        return -1;
    }

    int ret = -1;
    image_buffer_t dst_img;
    letterbox_t letter_box;
    rknn_input inputs[rknn_app_ctx_->io_num.n_input];
    rknn_output outputs[rknn_app_ctx_->io_num.n_output];
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
    rknn_app_ctx_->input_image_width = img->width;
    rknn_app_ctx_->input_image_height = img->height;
    dst_img.width = rknn_app_ctx_->model_width;
    dst_img.height = rknn_app_ctx_->model_height;
    dst_img.format = IMAGE_FORMAT_RGB888; // 确保 RGA/NPU 支持
    dst_img.size = get_image_size(&dst_img);

    if (posix_memalign((void **)&dst_img.virt_addr, 64, dst_img.size) != 0)
    {
        printf("failed to allocate aligned memory for dst_img\n");
        return -1;
    }
    memset(dst_img.virt_addr, bg_color, dst_img.size); // 初始化背景

    // -----------------------------
    // Step 2: 检查输入格式
    // -----------------------------
    image_buffer_t src_img_aligned = *img;

    if (img->format == IMAGE_FORMAT_RGBA8888)
    {
        // 将 RGBA -> RGB（手动逐像素转换）
        unsigned char *src = img->virt_addr;
        unsigned char *dst = (unsigned char *)malloc(img->width * img->height * 3);
        if (!dst)
        {
            printf("malloc failed for RGB conversion\n");
            free(dst_img.virt_addr);
            return -1;
        }

        for (int i = 0; i < img->width * img->height; i++)
        {
            dst[i * 3 + 0] = src[i * 4 + 0]; // R
            dst[i * 3 + 1] = src[i * 4 + 1]; // G
            dst[i * 3 + 2] = src[i * 4 + 2]; // B
        }

        src_img_aligned.virt_addr = dst;
        src_img_aligned.format = IMAGE_FORMAT_RGB888;
        src_img_aligned.size = img->width * img->height * 3;
    }

    // -----------------------------
    // Step 3: letterbox 预处理
    // -----------------------------
    ret = convert_image_with_letterbox(&src_img_aligned, &dst_img, &letter_box, bg_color);

    if (ret < 0)
    {
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
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = dst_img.width * dst_img.height * 3; // RGB888
    inputs[0].buf = dst_img.virt_addr;

    ret = rknn_inputs_set(rknn_app_ctx_->rknn_ctx, rknn_app_ctx_->io_num.n_input, inputs);
    if (ret < 0)
    {
        printf("rknn_inputs_set fail! ret=%d\n", ret);
        goto out;
    }

    // -----------------------------
    // Step 5: 推理
    // -----------------------------
    ret = rknn_run(rknn_app_ctx_->rknn_ctx, nullptr);
    if (ret < 0)
    {
        printf("rknn_run fail! ret=%d\n", ret);
        goto out;
    }

    // -----------------------------
    // Step 6: 获取输出
    // -----------------------------
    for (int i = 0; i < rknn_app_ctx_->io_num.n_output; i++)
    {
        outputs[i].index = i;
        outputs[i].want_float = (!rknn_app_ctx_->is_quant);
    }

    ret = rknn_outputs_get(rknn_app_ctx_->rknn_ctx, rknn_app_ctx_->io_num.n_output, outputs, NULL);
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        goto out;
    }

    // -----------------------------
    // Step 7: 后处理
    // -----------------------------
    post_process(rknn_app_ctx_, outputs, &letter_box,
                 box_conf_threshold, nms_threshold, od_results);

    rknn_outputs_release(rknn_app_ctx_->rknn_ctx, rknn_app_ctx_->io_num.n_output, outputs);

out:
    if (img->format == IMAGE_FORMAT_RGBA8888)
        free(src_img_aligned.virt_addr);

    if (dst_img.virt_addr)
        free(dst_img.virt_addr);

    return ret;
}
