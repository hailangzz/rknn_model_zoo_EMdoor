#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <fstream>
#include <algorithm>
#include <dirent.h>
#include <sstream>

#include "rknn_api.h"

// ================= CPU =================
struct CpuStat
{
  long long user = 0, nice = 0, system = 0, idle = 0;
};

CpuStat read_cpu()
{
  std::ifstream f("/proc/stat");
  std::string cpu;
  CpuStat s;
  f >> cpu >> s.user >> s.nice >> s.system >> s.idle;
  return s;
}

double calc_cpu(const CpuStat &a, const CpuStat &b)
{
  long long idle = b.idle - a.idle;
  long long total = (b.user - a.user) + (b.nice - a.nice) + (b.system - a.system) + (b.idle - a.idle);
  return total == 0 ? 0 : (1.0 - (double)idle / total) * 100;
}

// ================= MEM =================
double get_mem_usage()
{
  std::ifstream f("/proc/meminfo");
  std::string key;
  long total = 0, free = 0;

  while (f >> key)
  {
    if (key == "MemTotal:")
      f >> total;
    else if (key == "MemAvailable:")
    {
      f >> free;
      break;
    }
    else
      f.ignore(1000, '\n');
  }

  return (1.0 - (double)free / total) * 100;
}

// ================= NPU =================
struct NpuLoad
{
  int core0 = 0, core1 = 0, core2 = 0;
  double avg = 0;
  double max = 0;
};

// 解析 RK3588 debugfs
NpuLoad parse_npu(const std::string &line)
{
  NpuLoad n;
  sscanf(line.c_str(),
         "NPU load: Core0: %d%%, Core1: %d%%, Core2: %d%%",
         &n.core0, &n.core1, &n.core2);

  n.avg = (n.core0 + n.core1 + n.core2) / 3.0;
  n.max = std::max({n.core0, n.core1, n.core2});
  return n;
}

NpuLoad get_npu_usage()
{
  std::ifstream f("/sys/kernel/debug/rknpu/load");
  if (!f.is_open())
    return {-1, -1, -1, -1, -1};

  std::string line;
  std::getline(f, line);
  return parse_npu(line);
}

// ================= Args =================
struct Args
{
  std::vector<std::string> models;
  std::vector<int> threads;
  int run_time = 20;
  int warmup = 3;
  std::string output = "report.csv";
};

// ================= parse =================
Args parse_args(int argc, char **argv)
{
  Args a;

  for (int i = 1; i < argc; i++)
  {
    std::string k = argv[i];

    if (k == "--models")
    {
      while (i + 1 < argc && argv[i + 1][0] != '-')
        a.models.push_back(argv[++i]);
    }
    else if (k == "--threads")
    {
      while (i + 1 < argc && argv[i + 1][0] != '-')
        a.threads.push_back(std::atoi(argv[++i]));
    }
    else if (k == "--time")
    {
      a.run_time = atoi(argv[++i]);
    }
    else if (k == "--output")
    {
      a.output = argv[++i];
    }
  }

  if (a.models.empty())
    a.models = {"model/liquid_960p.rknn"};

  if (a.threads.empty())
    a.threads = {1, 2, 3, 4, 5, 6, 7, 8};

  return a;
}

// ================= Stats =================
struct Stats
{
  std::atomic<int> count{0};
  std::atomic<long long> lat_sum{0};
};

// ================= Worker =================
struct Worker
{
  rknn_context ctx;
  int input_size;
  rknn_tensor_format fmt;
  bool running = true;
  Stats stats;

  Worker(const std::string &path)
  {
    FILE *fp = fopen(path.c_str(), "rb");
    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);
    rewind(fp);

    std::vector<char> model(size);
    fread(model.data(), 1, size, fp);
    fclose(fp);

    rknn_init(&ctx, model.data(), size, 0, nullptr);

    rknn_tensor_attr attr{};
    attr.index = 0;
    rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));

    fmt = attr.fmt;
    input_size = 1;
    for (int i = 0; i < attr.n_dims; i++)
      input_size *= attr.dims[i];
  }

  void run()
  {
    std::vector<uint8_t> input(input_size, 0);

    rknn_input in{};
    in.index = 0;
    in.buf = input.data();
    in.size = input_size;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = fmt;

    while (running)
    {
      auto t1 = std::chrono::high_resolution_clock::now();

      rknn_inputs_set(ctx, 1, &in);
      rknn_run(ctx, nullptr);

      auto t2 = std::chrono::high_resolution_clock::now();

      long long lat =
          std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

      stats.count++;
      stats.lat_sum += lat;
    }
  }

  void stop() { running = false; }
  void release() { rknn_destroy(ctx); }
};

// ================= TEST =================
void run_test(int th, const Args &args, std::ofstream &csv)
{

  std::string model = args.models[0];

  std::vector<std::shared_ptr<Worker>> ws;
  for (int i = 0; i < th; i++)
    ws.push_back(std::make_shared<Worker>(model));

  std::vector<std::thread> ts;
  for (auto &w : ws)
    ts.emplace_back(&Worker::run, w);

  std::this_thread::sleep_for(std::chrono::seconds(args.warmup));

  CpuStat prev = read_cpu();

  long long total_fps = 0;
  double cpu_sum = 0, mem_sum = 0, npu_sum = 0;
  double npu_peak = 0;
  int npu_cnt = 0;
  int last = 0;

  for (int i = 0; i < args.run_time; i++)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    CpuStat cur = read_cpu();
    double cpu = calc_cpu(prev, cur);
    prev = cur;

    double mem = get_mem_usage();
    NpuLoad npu = get_npu_usage();

    cpu_sum += cpu;
    mem_sum += mem;

    if (npu.avg >= 0)
    {
      npu_sum += npu.avg;
      npu_peak = std::max(npu_peak, npu.max);
      npu_cnt++;
    }

    int total = 0;
    for (auto &w : ws)
      total += w->stats.count;

    int fps = total - last;
    last = total;

    total_fps += fps;

    std::cout << "[T" << th << "] FPS=" << fps
              << " CPU=" << cpu
              << " MEM=" << mem
              << " NPU=" << npu.avg << "%" << "\n";
  }

  for (auto &w : ws)
    w->stop();
  for (auto &t : ts)
    t.join();

  double avg_fps = total_fps / (double)args.run_time;
  double cpu_avg = cpu_sum / args.run_time;
  double mem_avg = mem_sum / args.run_time;
  double npu_avg = (npu_cnt ? npu_sum / npu_cnt : -1);

  // ================= ⭐ 关键修复：写CSV =================
  csv << th << ","
      << avg_fps << ","
      << cpu_avg << ","
      << mem_avg << ","
      << npu_avg << ","
      << npu_peak << "\n";

  std::cout << "\n==== RESULT " << th << " ====\n";
  std::cout << "FPS=" << avg_fps << "\n";
  std::cout << "CPU=" << cpu_avg << "%\n";
  std::cout << "MEM=" << mem_avg << "%\n";
  std::cout << "NPU(avg)=" << npu_avg << "%\n";
}

// ================= MAIN =================
int main(int argc, char **argv)
{

  Args args = parse_args(argc, argv);

  std::ofstream csv(args.output);
  csv << "threads,fps,cpu,mem,npu_avg,npu_peak\n";

  std::cout << "==== RKNN Benchmark ====\n";

  for (auto t : args.threads)
    run_test(t, args, csv);

  csv.close();

  std::cout << "Saved: " << args.output << std::endl;
  return 0;
}