#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <fstream>

#include "rknn_api.h"

// ================= CPU监控 =================
struct CpuStat
{
  long long user = 0, nice = 0, system = 0, idle = 0;
};

CpuStat read_cpu()
{
  std::ifstream file("/proc/stat");
  std::string cpu;
  CpuStat s;
  file >> cpu >> s.user >> s.nice >> s.system >> s.idle;
  return s;
}

double calc_cpu_usage(const CpuStat &a, const CpuStat &b)
{
  long long idle = b.idle - a.idle;
  long long total =
      (b.user - a.user) +
      (b.nice - a.nice) +
      (b.system - a.system) +
      (b.idle - a.idle);

  if (total == 0)
    return 0;

  return (1.0 - (double)idle / total) * 100.0;
}

// ================= 参数 =================
struct Args
{
  std::vector<std::string> models;
  std::vector<int> threads;
  int run_time = 20;
  std::string output = "rknn_report.csv";
};

Args parse_args(int argc, char **argv)
{
  Args args;

  for (int i = 1; i < argc; i++)
  {
    std::string key = argv[i];

    if (key == "--models")
    {
      i++;
      while (i < argc && argv[i][0] != '-')
        args.models.push_back(argv[i++]);
      i--;
    }
    else if (key == "--threads")
    {
      i++;
      while (i < argc && argv[i][0] != '-')
        args.threads.push_back(std::stoi(argv[i++]));
      i--;
    }
    else if (key == "--time")
    {
      args.run_time = std::stoi(argv[++i]);
    }
    else if (key == "--output")
    {
      args.output = argv[++i];
    }
  }

  if (args.models.empty())
  {
    args.models = {"model/liquid_960p.rknn", "model/wire_960p.rknn"};
  }

  if (args.threads.empty())
  {
    args.threads = {1, 2, 3, 4};
  }

  return args;
}

// ================= 统计 =================
struct Stats
{
  std::atomic<int> count{0};
  std::atomic<long long> total_latency{0};
};

// ================= Worker =================
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
    {
      std::cerr << "Open model failed\n";
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

    // 自动分配NPU核
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

    std::cout << "[Thread " << thread_id << "] Input size=" << input_size << std::endl;

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

// ================= 测试 =================
void run_test(int thread_num,
              const std::vector<std::string> &models,
              int run_time,
              std::ofstream &csv)
{
  std::vector<std::shared_ptr<Worker>> workers;

  for (int i = 0; i < thread_num; i++)
  {
    auto w = std::make_shared<Worker>(models[i % models.size()], i);
    if (!w->init())
      return;
    workers.push_back(w);
  }

  std::vector<std::thread> threads;
  for (auto &w : workers)
    threads.emplace_back(&Worker::run, w);

  CpuStat prev = read_cpu();
  double cpu_sum = 0;

  for (int i = 0; i < run_time; i++)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    CpuStat curr = read_cpu();
    double cpu = calc_cpu_usage(prev, curr);
    prev = curr;
    cpu_sum += cpu;

    int total = 0;
    for (auto &w : workers)
      total += w->stats.count;

    std::cout << "[Threads " << thread_num << "] "
              << "Time " << i + 1
              << "s FPS=" << total / (i + 1)
              << " CPU=" << cpu << "%\n";
  }

  for (auto &w : workers)
    w->stop();
  for (auto &t : threads)
    t.join();

  int total_count = 0;
  long long total_latency = 0;

  for (auto &w : workers)
  {
    total_count += w->stats.count;
    total_latency += w->stats.total_latency;
  }

  double fps = total_count * 1.0 / run_time;
  double avg_latency = total_latency * 1.0 / total_count / 1000.0;
  double avg_cpu = cpu_sum / run_time;

  // 👉 NPU估算（简单模型）
  double npu_util = std::min(100.0, fps / (thread_num * 40.0) * 100.0);

  std::cout << "\n==== RESULT (" << thread_num << " threads) ====\n";
  std::cout << "FPS: " << fps << "\n";
  std::cout << "Latency: " << avg_latency << " ms\n";
  std::cout << "CPU: " << avg_cpu << " %\n";
  std::cout << "NPU(est): " << npu_util << " %\n";

  csv << thread_num << ","
      << fps << ","
      << avg_latency << ","
      << avg_cpu << ","
      << npu_util << "\n";

  for (auto &w : workers)
    w->release();
}

// ================= main =================
int main(int argc, char **argv)
{
  Args args = parse_args(argc, argv);

  std::ofstream csv(args.output);
  csv << "threads,fps,latency_ms,cpu,npu_est\n";

  std::cout << "==== RKNN Stress Test ====\n";

  for (auto t : args.threads)
  {
    run_test(t, args.models, args.run_time, csv);
  }

  csv.close();

  std::cout << "\nReport saved: " << args.output << std::endl;
  return 0;
}