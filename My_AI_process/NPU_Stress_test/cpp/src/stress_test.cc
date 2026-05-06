#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <fstream>
#include <mutex>

#include "rknn_api.h"

struct Stats
{
  std::atomic<int> count{0};
  std::atomic<long long> total_latency{0};
};

struct Worker
{
  std::string model_path;
  int thread_id;

  rknn_context ctx;
  int input_size;
  rknn_tensor_format input_fmt;

  Stats stats;
  bool running;

  Worker(const std::string &path, int id)
      : model_path(path), thread_id(id), running(true) {}

  bool init()
  {
    FILE *fp = fopen(model_path.c_str(), "rb");
    if (!fp)
      return false;

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);
    rewind(fp);

    std::vector<char> model(size);
    fread(model.data(), 1, size, fp);
    fclose(fp);

    if (rknn_init(&ctx, model.data(), size, 0, NULL) != RKNN_SUCC)
      return false;

    // 👉 自动分配NPU核
    rknn_core_mask mask;
    if (thread_id % 3 == 0)
      mask = RKNN_NPU_CORE_0;
    else if (thread_id % 3 == 1)
      mask = RKNN_NPU_CORE_1;
    else
      mask = RKNN_NPU_CORE_2;

    rknn_set_core_mask(ctx, mask);

    // 查询输入
    rknn_tensor_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.index = 0;
    rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));

    input_fmt = attr.fmt;

    input_size = 1;
    for (int i = 0; i < attr.n_dims; i++)
      input_size *= attr.dims[i];

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
      auto t1 = std::chrono::high_resolution_clock::now();

      if (rknn_inputs_set(ctx, 1, inputs) != RKNN_SUCC)
        break;
      if (rknn_run(ctx, NULL) != RKNN_SUCC)
        break;

      auto t2 = std::chrono::high_resolution_clock::now();

      long long latency =
          std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

      stats.count++;
      stats.total_latency += latency;
    }
  }

  void stop() { running = false; }

  void release() { rknn_destroy(ctx); }
};

void run_test(int thread_num,
              const std::vector<std::string> &models,
              int run_time,
              std::ofstream &csv)
{

  std::vector<std::shared_ptr<Worker>> workers;

  for (int i = 0; i < thread_num; i++)
  {
    auto w = std::make_shared<Worker>(
        models[i % models.size()], i);

    if (!w->init())
    {
      std::cerr << "Init failed\n";
      return;
    }
    workers.push_back(w);
  }

  std::vector<std::thread> threads;
  for (auto &w : workers)
    threads.emplace_back(&Worker::run, w);

  // 👉 实时FPS打印
  for (int i = 0; i < run_time; i++)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    int total = 0;
    for (auto &w : workers)
      total += w->stats.count;

    std::cout << "[Threads " << thread_num << "] "
              << "Time " << i + 1 << "s, FPS=" << total / (i + 1) << std::endl;
  }

  for (auto &w : workers)
    w->stop();
  for (auto &t : threads)
    t.join();

  // 👉 汇总
  int total_count = 0;
  long long total_latency = 0;

  for (auto &w : workers)
  {
    total_count += w->stats.count;
    total_latency += w->stats.total_latency;
  }

  double fps = total_count * 1.0 / run_time;
  double avg_latency = total_latency * 1.0 / total_count / 1000.0;

  std::cout << "\n==== RESULT (Threads=" << thread_num << ") ====\n";
  std::cout << "Total FPS: " << fps << std::endl;
  std::cout << "Avg Latency: " << avg_latency << " ms\n";

  // 写CSV
  csv << thread_num << "," << fps << "," << avg_latency << "\n";

  for (auto &w : workers)
    w->release();
}

int main()
{
  std::vector<std::string> models = {
      "model/liquid_960p.rknn",
      "model/wire_960p.rknn"};

  int run_time = 20;

  std::ofstream csv("rknn_report.csv");
  csv << "threads,fps,avg_latency(ms)\n";

  std::cout << "==== RKNN Auto Stress Test ====\n";

  // 👉 自动跑1~4线程
  for (int t = 1; t <= 4; t++)
  {
    run_test(t, models, run_time, csv);
  }

  csv.close();

  std::cout << "\n📄 Report saved: rknn_report.csv\n";

  return 0;
}