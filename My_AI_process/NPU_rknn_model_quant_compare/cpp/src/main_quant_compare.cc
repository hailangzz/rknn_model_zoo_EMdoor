#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>

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

// =========================
// metrics
// =========================
static tensor_metrics_t calc_metrics(
    const float *a,
    const float *b,
    int count)
{
  tensor_metrics_t m;
  memset(&m, 0, sizeof(m));

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
      max_err = fabs(d);
  }

  m.mse = mse / count;
  m.mae = mae / count;

  double norm_a = sqrt(na);
  double norm_b = sqrt(nb);

  // ============================
  // ⭐关键修复点：避免“0向量cosine异常”
  // ============================
  if (norm_a < 1e-8 || norm_b < 1e-8)
  {
    m.cosine = 1.0; // 或者设为 -1 / NaN / skip标志
  }
  else
  {
    m.cosine = dot / (norm_a * norm_b);
  }

  m.max_err = max_err;

  return m;
}

// =========================
// SAFE inference
// =========================
static int run_model(
    rknn_app_context_t *app_ctx,
    const image_buffer_t *src_img,
    std::vector<std::vector<float>> &all_outputs)
{
  int ret;

  // =========================
  // 1. preprocess (NO pollution)
  // =========================
  image_buffer_t dst_img;
  memset(&dst_img, 0, sizeof(dst_img));

  dst_img.width = app_ctx->model_width;
  dst_img.height = app_ctx->model_height;
  dst_img.format = IMAGE_FORMAT_RGB888;
  dst_img.size = get_image_size(&dst_img);

  dst_img.virt_addr = (unsigned char *)malloc(dst_img.size);
  if (!dst_img.virt_addr)
  {
    printf("malloc failed\n");
    return -1;
  }

  letterbox_t letter_box;
  memset(&letter_box, 0, sizeof(letter_box));

  image_buffer_t tmp_src;
  memcpy(&tmp_src, src_img, sizeof(image_buffer_t));

  ret = convert_image_with_letterbox(
      &tmp_src,
      &dst_img,
      &letter_box,
      114);

  if (ret < 0)
  {
    free(dst_img.virt_addr);
    return -1;
  }

  // =========================
  // 2. input
  // =========================
  rknn_input inputs[1];
  memset(inputs, 0, sizeof(inputs));

  inputs[0].index = 0;
  inputs[0].fmt = RKNN_TENSOR_NHWC;
  inputs[0].buf = dst_img.virt_addr;
  inputs[0].size = dst_img.size;

  // 🔥 强制 UINT8（保证 FP32 vs INT8 公平）
  inputs[0].type = RKNN_TENSOR_UINT8;

  ret = rknn_inputs_set(app_ctx->rknn_ctx, 1, inputs);
  if (ret < 0)
  {
    free(dst_img.virt_addr);
    return -1;
  }

  // =========================
  // 3. run
  // =========================
  ret = rknn_run(app_ctx->rknn_ctx, NULL);
  if (ret < 0)
  {
    free(dst_img.virt_addr);
    return -1;
  }

  // =========================
  // 4. output
  // =========================
  std::vector<rknn_output> outputs(app_ctx->io_num.n_output);
  memset(outputs.data(), 0, sizeof(rknn_output) * outputs.size());

  for (int i = 0; i < app_ctx->io_num.n_output; i++)
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
    free(dst_img.virt_addr);
    return -1;
  }

  // =========================
  // 5. copy outputs safely
  // =========================
  all_outputs.clear();
  all_outputs.reserve(app_ctx->io_num.n_output);

  for (int i = 0; i < app_ctx->io_num.n_output; i++)
  {
    int count = app_ctx->output_attrs[i].n_elems;
    float *ptr = (float *)outputs[i].buf;

    std::vector<float> tensor(ptr, ptr + count);
    all_outputs.emplace_back(std::move(tensor));
  }

  rknn_outputs_release(
      app_ctx->rknn_ctx,
      app_ctx->io_num.n_output,
      outputs.data());

  free(dst_img.virt_addr);

  return 0;
}

// =========================
// main
// =========================
int main(int argc, char **argv)
{
  if (argc != 4)
  {
    printf("Usage:\n%s fp32.rknn int8.rknn image.jpg\n", argv[0]);
    return -1;
  }

  const char *fp32_model = argv[1];
  const char *int8_model = argv[2];
  const char *image_path = argv[3];

  printf("RKNN FP32 vs INT8 compare demo\n");

  // =========================
  // init models
  // =========================
  rknn_app_context_t fp_ctx;
  rknn_app_context_t int8_ctx;

  memset(&fp_ctx, 0, sizeof(fp_ctx));
  memset(&int8_ctx, 0, sizeof(int8_ctx));

  if (init_yolov8_seg_model(fp32_model, &fp_ctx) != 0)
  {
    printf("fp32 init failed\n");
    return -1;
  }

  if (init_yolov8_seg_model(int8_model, &int8_ctx) != 0)
  {
    printf("int8 init failed\n");
    return -1;
  }

  // =========================
  // load image
  // =========================
  image_buffer_t src_img;
  memset(&src_img, 0, sizeof(src_img));

  if (read_image(image_path, &src_img) != 0)
  {
    printf("read image failed\n");
    return -1;
  }

  // =========================
  // inference
  // =========================
  std::vector<std::vector<float>> fp_out;
  std::vector<std::vector<float>> int8_out;

  printf("\n===== FP32 =====\n");
  run_model(&fp_ctx, &src_img, fp_out);

  printf("\n===== INT8 =====\n");
  run_model(&int8_ctx, &src_img, int8_out);

  // =========================
  // compare
  // =========================
  printf("\n===== COMPARE =====\n");

  size_t n = std::min(fp_out.size(), int8_out.size());

  for (size_t i = 0; i < n; i++)
  {
    if (fp_out[i].size() != int8_out[i].size())
    {
      printf("skip output %zu (size mismatch)\n", i);
      continue;
    }

    tensor_metrics_t m = calc_metrics(
        fp_out[i].data(),
        int8_out[i].data(),
        fp_out[i].size());

    printf("\nOUTPUT %zu\n", i);
    printf("MSE     : %.6f\n", m.mse);
    printf("MAE     : %.6f\n", m.mae);
    printf("COSINE  : %.6f\n", m.cosine);
    printf("MAXERR  : %.6f\n", m.max_err);
  }

  // =========================
  // release
  // =========================
  release_yolov8_seg_model(&fp_ctx);
  release_yolov8_seg_model(&int8_ctx);

  if (src_img.virt_addr)
  {
    free(src_img.virt_addr);
  }

  return 0;
}