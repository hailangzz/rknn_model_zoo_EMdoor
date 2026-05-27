#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>

#include "rknn_api.h"
#include "image_utils.h"
#include "yolov8_seg.h"

// =========================
// metrics
// =========================
typedef struct
{
  double mse;
  double mae;
  double cosine;
  double max_err;
} tensor_metrics_t;

static tensor_metrics_t calc_metrics(float *a, float *b, int count)
{
  tensor_metrics_t m = {0};

  double dot = 0, na = 0, nb = 0;

  for (int i = 0; i < count; i++)
  {
    double d = a[i] - b[i];

    m.mse += d * d;
    m.mae += fabs(d);

    dot += a[i] * b[i];
    na += a[i] * a[i];
    nb += b[i] * b[i];

    if (fabs(d) > m.max_err)
      m.max_err = fabs(d);
  }

  m.mse /= count;
  m.mae /= count;
  m.cosine = dot / (sqrt(na) * sqrt(nb) + 1e-9);

  return m;
}

// =========================
// run model (IMPORTANT FIX)
// =========================
static int run_model(
    rknn_app_context_t *app_ctx,
    image_buffer_t *src_img)
{
  object_detect_result_list od_results;

  // ❗关键修复：确保每次推理都用干净输入（防止FP32污染INT8）
  image_buffer_t local_img;
  memset(&local_img, 0, sizeof(local_img));

  local_img.width = src_img->width;
  local_img.height = src_img->height;
  local_img.format = src_img->format;
  local_img.size = src_img->size;

  local_img.virt_addr = (unsigned char *)malloc(local_img.size);
  memcpy(local_img.virt_addr, src_img->virt_addr, local_img.size);

  int ret = inference_yolov8_seg_model(app_ctx, &local_img, &od_results);

  if (ret != 0)
  {
    printf("inference fail ret=%d\n", ret);
    free(local_img.virt_addr);
    return -1;
  }

  printf("detect count = %d\n", od_results.count);

  // =========================
  // draw mask (safe version)
  // =========================
  if (od_results.count > 0 && od_results.results_seg[0].seg_mask)
  {
    int w = src_img->width;
    int h = src_img->height;

    char *img = (char *)src_img->virt_addr;
    uint8_t *mask = od_results.results_seg[0].seg_mask;

    for (int y = 0; y < h; y++)
    {
      for (int x = 0; x < w; x++)
      {
        if (mask[y * w + x])
        {
          int idx = 3 * (y * w + x);
          img[idx] = 0;
          img[idx + 1] = 255;
          img[idx + 2] = 0;
        }
      }
    }

    free(mask);
  }

  free(local_img.virt_addr);
  return od_results.count;
}

// =========================
// main
// =========================
int main(int argc, char **argv)
{
  if (argc != 4)
  {
    printf("Usage: %s fp.rknn int8.rknn image.jpg\n", argv[0]);
    return -1;
  }

  rknn_app_context_t fp_ctx;
  rknn_app_context_t int8_ctx;

  memset(&fp_ctx, 0, sizeof(fp_ctx));
  memset(&int8_ctx, 0, sizeof(int8_ctx));

  if (init_yolov8_seg_model(argv[1], &fp_ctx) != 0)
  {
    printf("fp init fail\n");
    return -1;
  }

  if (init_yolov8_seg_model(argv[2], &int8_ctx) != 0)
  {
    printf("int8 init fail\n");
    return -1;
  }

  image_buffer_t src_img;
  memset(&src_img, 0, sizeof(src_img));

  if (read_image(argv[3], &src_img) != 0)
  {
    printf("read image fail\n");
    return -1;
  }

  // =========================
  // IMPORTANT FIX:
  // FP32 / INT8 MUST NOT SHARE MODIFIED IMAGE
  // =========================

  image_buffer_t fp_img;
  image_buffer_t int8_img;

  memcpy(&fp_img, &src_img, sizeof(image_buffer_t));
  memcpy(&int8_img, &src_img, sizeof(image_buffer_t));

  fp_img.virt_addr = (unsigned char *)malloc(src_img.size);
  int8_img.virt_addr = (unsigned char *)malloc(src_img.size);

  memcpy(fp_img.virt_addr, src_img.virt_addr, src_img.size);
  memcpy(int8_img.virt_addr, src_img.virt_addr, src_img.size);

  printf("\n===== FP32 =====\n");
  int fp_count = run_model(&fp_ctx, &fp_img);

  printf("\n===== INT8 =====\n");
  int int8_count = run_model(&int8_ctx, &int8_img);

  printf("\nRESULT:\n");
  printf("FP32  count = %d\n", fp_count);
  printf("INT8  count = %d\n", int8_count);

  // =========================
  // release
  // =========================
  release_yolov8_seg_model(&fp_ctx);
  release_yolov8_seg_model(&int8_ctx);

  free(src_img.virt_addr);
  free(fp_img.virt_addr);
  free(int8_img.virt_addr);

  return 0;
}