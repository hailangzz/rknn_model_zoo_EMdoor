#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>

#include "rknn_api.h"

struct Worker
{
  std::string model_path;
  int core_mask;
  int thread_id;
  std::atomic<int> count;
  bool running;

  rknn_context ctx;

  // 自动获取的输入信息
  int input_size;
  rknn_tensor_format input_fmt;

  Worker(const std::string &path, int core, int id)
      : model_path(path), core_mask(core), thread_id(id),
        count(0), running(true), input_size(0) {}

  bool init()
  {
    FILE *fp = fopen(model_path.c_str(), "rb");
    if (!fp)
    {
      std::cerr << "Failed to open model: " << model_path << std::endl;
      return false;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);
    rewind(fp);

    std::vector<char> model(size);
    fread(model.data(), 1, size, fp);
    fclose(fp);

    if (rknn_init(&ctx, model.data(), size, 0, NULL) != RKNN_SUCC)
    {
      std::cerr << "rknn_init failed\n";
      return false;
    }

    // 👉 设置单核（先保证稳定）
    rknn_core_mask mask = RKNN_NPU_CORE_0;
    if (rknn_set_core_mask(ctx, mask) != RKNN_SUCC)
    {
      std::cerr << "set core mask failed\n";
    }

    // 👉 查询输入信息（关键！）
    rknn_input_output_num io_num;
    if (rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)) != RKNN_SUCC)
    {
      std::cerr << "query io num failed\n";
      return false;
    }

    rknn_tensor_attr input_attr;
    memset(&input_attr, 0, sizeof(input_attr));
    input_attr.index = 0;

    if (rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &input_attr, sizeof(input_attr)) != RKNN_SUCC)
    {
      std::cerr << "query input attr failed\n";
      return false;
    }

    std::cout << "[Thread " << thread_id << "] Input dims: ";
    for (int i = 0; i < input_attr.n_dims; i++)
    {
      std::cout << input_attr.dims[i] << " ";
    }
    std::cout << std::endl;

    input_fmt = input_attr.fmt;

    // 👉 自动计算 size
    input_size = 1;
    for (int i = 0; i < input_attr.n_dims; i++)
    {
      input_size *= input_attr.dims[i];
    }

    std::cout << "[Thread " << thread_id << "] Input size: " << input_size << std::endl;
    std::cout << "[Thread " << thread_id << "] Format: "
              << (input_fmt == RKNN_TENSOR_NCHW ? "NCHW" : "NHWC") << std::endl;

    return true;
  }

  void run()
  {
    std::vector<uint8_t> input(input_size, 0);

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].buf = input.data();
    inputs[0].size = input_size;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = input_fmt;

    while (running)
    {
      if (rknn_inputs_set(ctx, 1, inputs) != RKNN_SUCC)
      {
        std::cerr << "[Thread " << thread_id << "] inputs_set failed\n";
        break;
      }

      if (rknn_run(ctx, NULL) != RKNN_SUCC)
      {
        std::cerr << "[Thread " << thread_id << "] rknn_run failed\n";
        break;
      }

      count++;
    }
  }

  void stop()
  {
    running = false;
  }

  void release()
  {
    rknn_destroy(ctx);
  }
};

int main()
{
  std::vector<std::string> models = {
      "model/liquid_960p.rknn",
      "model/wire_960p.rknn"};

  int thread_num = 1; // 👉 先用1线程稳定
  int run_time = 30;

  std::vector<std::shared_ptr<Worker>> workers;

  std::cout << "==== RKNN C++ Stress Test (Safe Mode) ====\n";

  // 初始化
  for (int i = 0; i < thread_num; i++)
  {
    auto w = std::make_shared<Worker>(
        models[i % models.size()],
        0,
        i);

    if (!w->init())
    {
      std::cerr << "Init failed\n";
      return -1;
    }

    workers.push_back(w);
  }

  std::vector<std::thread> threads;

  // 启动线程
  for (auto &w : workers)
  {
    threads.emplace_back(&Worker::run, w);
  }

  auto start = std::chrono::steady_clock::now();
  std::this_thread::sleep_for(std::chrono::seconds(run_time));

  for (auto &w : workers)
  {
    w->stop();
  }

  for (auto &t : threads)
  {
    t.join();
  }

  auto end = std::chrono::steady_clock::now();
  double total_time = std::chrono::duration<double>(end - start).count();

  int total_count = 0;
  for (int i = 0; i < workers.size(); i++)
  {
    double fps = workers[i]->count / total_time;
    std::cout << "Thread-" << i << ": " << fps << " FPS\n";
    total_count += workers[i]->count;
  }

  std::cout << "------------------\n";
  std::cout << "Total FPS: " << total_count / total_time << std::endl;

  for (auto &w : workers)
  {
    w->release();
  }

  return 0;
}