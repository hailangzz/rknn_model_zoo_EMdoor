#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>

#include "rknn_api.h"
#include "image_utils.h"
#include "yolov8_seg.h"

typedef struct
{
  double mse;
  double mae;
  double cosine;
  double max_err;

} tensor_metrics_t;

static tensor_metrics_t calc_metrics(
    float *a,
    float *b,
    int count)
{
  tensor_metrics_t m;

  double mse = 0;
  double mae = 0;

  double dot = 0;
  double na = 0;
  double nb = 0;

  double max_err = 0;

  for (int i = 0; i < count; i++)
  {
    double d = a[i] - b[i];

    mse += d * d;

    mae += fabs(d);

    dot += a[i] * b[i];

    na += a[i] * a[i];

    nb += b[i] * b[i];

    if (fabs(d) > max_err)
    {
      max_err = fabs(d);
    }
  }

  m.mse = mse / count;

  m.mae = mae / count;

  m.cosine =
      dot / (sqrt(na) * sqrt(nb) + 1e-9);

  m.max_err = max_err;

  return m;
}

static int run_model(
    rknn_app_context_t *app_ctx,
    image_buffer_t *src_img,
    std::vector<std::vector<float>> &all_outputs)
{
  int ret;

  image_buffer_t dst_img;
  memset(&dst_img, 0, sizeof(dst_img));

  dst_img.width = app_ctx->model_width;
  dst_img.height = app_ctx->model_height;

  dst_img.format = IMAGE_FORMAT_RGB888;

  dst_img.size = get_image_size(&dst_img);

  dst_img.virt_addr =
      (unsigned char *)malloc(dst_img.size);

  if (!dst_img.virt_addr)
  {
    printf("malloc dst_img failed\n");
    return -1;
  }

  letterbox_t letter_box;
  memset(&letter_box, 0, sizeof(letter_box));

  ret = convert_image_with_letterbox(
      src_img,
      &dst_img,
      &letter_box,
      114);

  if (ret < 0)
  {
    printf("convert_image_with_letterbox fail\n");

    free(dst_img.virt_addr);

    return -1;
  }

  rknn_input inputs[1];

  memset(inputs, 0, sizeof(inputs));

  inputs[0].index = 0;

  // 关键
  if (app_ctx->input_attrs[0].type == RKNN_TENSOR_FLOAT16)
  {
    inputs[0].type = RKNN_TENSOR_FLOAT16;
  }
  else
  {
    inputs[0].type = RKNN_TENSOR_UINT8;
  }

  inputs[0].fmt = RKNN_TENSOR_NHWC;

  inputs[0].size = app_ctx->input_attrs[0].size;

  inputs[0].buf = dst_img.virt_addr;

  ret = rknn_inputs_set(
      app_ctx->rknn_ctx,
      app_ctx->io_num.n_input,
      inputs);

  if (ret < 0)
  {
    printf("rknn_inputs_set fail\n");

    free(dst_img.virt_addr);

    return -1;
  }

  ret = rknn_run(app_ctx->rknn_ctx, NULL);

  if (ret < 0)
  {
    printf("rknn_run fail\n");

    free(dst_img.virt_addr);

    return -1;
  }

  std::vector<rknn_output> outputs(
      app_ctx->io_num.n_output);

  memset(outputs.data(),
         0,
         sizeof(rknn_output) *
             app_ctx->io_num.n_output);

  for (int i = 0;
       i < app_ctx->io_num.n_output;
       i++)
  {
    outputs[i].index = i;

    outputs[i].want_float = 1;
  }

  ret = rknn_outputs_get(
      app_ctx->rknn_ctx,
      app_ctx->io_num.n_output,
      outputs.data(),
      NULL);

  if (ret < 0)
  {
    printf("rknn_outputs_get fail\n");

    free(dst_img.virt_addr);

    return -1;
  }

  for (int i = 0;
       i < app_ctx->io_num.n_output;
       i++)
  {
    int count =
        app_ctx->output_attrs[i].n_elems;

    float *ptr =
        (float *)outputs[i].buf;

    std::vector<float> tensor(
        ptr,
        ptr + count);

    all_outputs.push_back(tensor);
  }

  rknn_outputs_release(
      app_ctx->rknn_ctx,
      app_ctx->io_num.n_output,
      outputs.data());

  free(dst_img.virt_addr);

  return 0;
}

int main(int argc, char **argv)
{
  if (argc != 4)
  {
    printf("\nUsage:\n");

    printf("%s fp32.rknn int8.rknn image.jpg\n",
           argv[0]);

    return -1;
  }

  const char *fp32_model = argv[1];

  const char *int8_model = argv[2];

  const char *image_path = argv[3];

  printf("RKNN quant compare demo\n");

  // ==========================================
  // init fp32
  // ==========================================

  rknn_app_context_t fp32_ctx;

  memset(&fp32_ctx, 0, sizeof(fp32_ctx));

  if (init_yolov8_seg_model(
          fp32_model,
          &fp32_ctx) != 0)
  {
    printf("init fp32 model failed\n");

    return -1;
  }

  // ==========================================
  // init int8
  // ==========================================

  rknn_app_context_t int8_ctx;

  memset(&int8_ctx, 0, sizeof(int8_ctx));

  if (init_yolov8_seg_model(
          int8_model,
          &int8_ctx) != 0)
  {
    printf("init int8 model failed\n");

    return -1;
  }

  // ==========================================
  // read image
  // ==========================================

  image_buffer_t src_img;

  memset(&src_img, 0, sizeof(src_img));

  if (read_image(image_path,
                 &src_img) != 0)
  {
    printf("read image fail\n");

    return -1;
  }

  // ==========================================
  // inference
  // ==========================================

  std::vector<std::vector<float>> fp32_outputs;

  std::vector<std::vector<float>> int8_outputs;

  printf("\n===== FP32 =====\n");

  if (run_model(
          &fp32_ctx,
          &src_img,
          fp32_outputs) != 0)
  {
    return -1;
  }

  printf("\n===== INT8 =====\n");

  if (run_model(
          &int8_ctx,
          &src_img,
          int8_outputs) != 0)
  {
    return -1;
  }

  // ==========================================
  // compare
  // ==========================================

  printf("\n===== COMPARE =====\n");

  for (size_t i = 0;
       i < fp32_outputs.size();
       i++)
  {
    tensor_metrics_t m =
        calc_metrics(
            fp32_outputs[i].data(),
            int8_outputs[i].data(),
            fp32_outputs[i].size());

    printf("\nOUTPUT %zu\n", i);

    printf("MSE           : %.10f\n",
           m.mse);

    printf("MAE           : %.10f\n",
           m.mae);

    printf("COSINE        : %.10f\n",
           m.cosine);

    printf("MAX ERROR     : %.10f\n",
           m.max_err);
  }

  // ==========================================
  // release
  // ==========================================

  release_yolov8_seg_model(&fp32_ctx);

  release_yolov8_seg_model(&int8_ctx);

  if (src_img.virt_addr)
  {
    free(src_img.virt_addr);
  }

  return 0;
}