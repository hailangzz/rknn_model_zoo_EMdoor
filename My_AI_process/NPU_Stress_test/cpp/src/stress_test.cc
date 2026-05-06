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
  long long total = (b.user - a.user) + (b.nice - a.nice) +
                    (b.system - a.system) + (b.idle - a.idle);

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
  int core0 = 0;
  int core1 = 0;
  int core2 = 0;
  double avg = 0;
  double max = 0;
};

NpuLoad parse_npu_line(const std::string &line)
{
  NpuLoad npu;

  sscanf(line.c_str(),
         "NPU load: Core0: %d%%, Core1: %d%%, Core2: %d%%",
         &npu.core0,
         &npu.core1,
         &npu.core2);

  npu.avg = (npu.core0 + npu.core1 + npu.core2) / 3.0;
  npu.max = std::max({npu.core0, npu.core1, npu.core2});

  return npu;
}

NpuLoad get_npu_usage()
{
  std::ifstream f("/sys/kernel/debug/rknpu/load");
  if (!f.is_open())
    return {-1, -1, -1, -1, -1};

  std::string line;
  std::getline(f, line);

  return parse_npu_line(line);
}

// ================= Args（升级版）=================
struct Args
{
  std::vector<std::string> models;
  std::vector<int> threads;
  int run_time = 30;
  int warmup = 3;
  std::string output = "report.csv";
};

// ================= 参数解析 =================
Args parse_args(int argc, char **argv)
{
  Args args;

  for (int i = 1; i < argc; i++)
  {
    std::string key = argv[i];

    // -------- models --------
    if (key == "--models")
    {
      while (i + 1 < argc && argv[i + 1][0] != '-')
      {
        args.models.push_back(argv[++i]);
      }
    }

    // -------- threads --------
    else if (key == "--threads")
    {
      while (i + 1 < argc && argv[i + 1][0] != '-')
      {
        args.threads.push_back(std::atoi(argv[++i]));
      }
    }

    // -------- time --------
    else if (key == "--time")
    {
      args.run_time = std::atoi(argv[++i]);
    }

    // -------- output --------
    else if (key == "--output")
    {
      args.output = argv[++i];
    }
  }

  // 默认值保护
  if (args.models.empty())
    args.models.push_back("model/liquid_960p.rknn");

  if (args.threads.empty())
    args.threads = {1, 2, 3, 4};

  return args;
}

// ================= Stats =================
struct Stats
{
  std::atomic<int> count{0};
  std::atomic<long long> latency_sum{0};
  std::vector<long long> lat_vec;
};

// ================= Worker =================
struct Worker
{
  rknn_context ctx;
  int input_size;
  rknn_tensor_format fmt;
  int id;
  bool running = true;
  Stats stats;

  Worker(std::string path, int tid) : id(tid)
  {
    FILE *fp = fopen(path.c_str(), "rb");
    if (!fp)
    {
      std::cerr << "open model failed: " << path << "\n";
      exit(-1);
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);
    rewind(fp);

    std::vector<char> model(size);
    fread(model.data(), 1, size, fp);
    fclose(fp);

    rknn_init(&ctx, model.data(), size, 0, NULL);

    rknn_tensor_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.index = 0;
    rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));

    fmt = attr.fmt;

    input_size = 1;
    for (int i = 0; i < attr.n_dims; i++)
      input_size *= attr.dims[i];

    std::cout << "Thread " << id << " input=" << input_size << "\n";
  }

  void run()
  {
    std::vector<uint8_t> input(input_size, 0);

    rknn_input in[1] = {0};
    in[0].index = 0;
    in[0].buf = input.data();
    in[0].size = input_size;
    in[0].type = RKNN_TENSOR_UINT8;
    in[0].fmt = fmt;

    while (running)
    {
      auto t1 = std::chrono::high_resolution_clock::now();

      rknn_inputs_set(ctx, 1, in);
      rknn_run(ctx, NULL);

      auto t2 = std::chrono::high_resolution_clock::now();

      long long lat =
          std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

      stats.count++;
      stats.latency_sum += lat;

      if (stats.lat_vec.size() < 10000)
        stats.lat_vec.push_back(lat);
    }
  }

  void stop() { running = false; }
  void release() { rknn_destroy(ctx); }
};

// ================= percentile =================
double percentile(std::vector<long long> &v, double p)
{
  if (v.empty())
    return 0;
  std::sort(v.begin(), v.end());
  int idx = std::min((int)(p * v.size()), (int)v.size() - 1);
  return v[idx] / 1000.0;
}

// ================= test =================
void run_test(int th, const Args &args, std::ofstream &csv)
{
  std::vector<std::shared_ptr<Worker>> ws;

  std::string model = args.models[th % args.models.size()];

  for (int i = 0; i < th; i++)
    ws.push_back(std::make_shared<Worker>(model, i));

  std::vector<std::thread> ts;
  for (auto &w : ws)
    ts.emplace_back(&Worker::run, w);

  std::this_thread::sleep_for(std::chrono::seconds(args.warmup));

  CpuStat prev = read_cpu();

  double cpu_sum = 0, mem_sum = 0;
  double npu_sum = 0;
  double npu_peak = 0;
  int npu_cnt = 0;

  int last_total = 0;

  for (int t = 0; t < args.run_time; t++)
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

    int fps = total - last_total;
    last_total = total;

    std::cout << "[T" << th << "] FPS=" << fps
              << " CPU=" << cpu
              << " MEM=" << mem
              << " NPU(avg)=" << npu.avg
              << " max=" << npu.max << "\n";
  }

  for (auto &w : ws)
    w->stop();
  for (auto &t : ts)
    t.join();

  std::cout << "\n==== RESULT " << th << " DONE ====\n";
}

// ================= main =================
int main(int argc, char **argv)
{
  Args args = parse_args(argc, argv);

  std::cout << "==== RKNN Benchmark (CLI UPGRADED) ====\n";

  std::ofstream csv(args.output);
  csv << "threads,fps,avg_ms,p50,p90,p99,cpu,mem,npu_avg,npu_peak\n";

  for (auto t : args.threads)
    run_test(t, args, csv);

  csv.close();

  std::cout << "Report saved: " << args.output << std::endl;

  return 0;
}