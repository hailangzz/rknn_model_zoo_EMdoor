#include <android/log.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>

#include "rknn_api.h"

#define LOG_TAG "RKNN_BENCH"

#define LOGI(...) \
  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

#define LOGE(...) \
  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

// =====================================================
// CPU
// =====================================================

struct CpuStat
{
  long long user = 0;
  long long nice = 0;
  long long system = 0;
  long long idle = 0;
};

CpuStat read_cpu()
{
  CpuStat s;

  std::ifstream f("/proc/stat");

  if (!f.is_open())
    return s;

  std::string cpu;

  f >> cpu >> s.user >> s.nice >> s.system >> s.idle;

  return s;
}

double calc_cpu(const CpuStat &a, const CpuStat &b)
{
  long long idle =
      b.idle - a.idle;

  long long total =
      (b.user - a.user) +
      (b.nice - a.nice) +
      (b.system - a.system) +
      (b.idle - a.idle);

  if (total <= 0)
    return 0;

  return (1.0 - (double)idle / total) * 100.0;
}

// =====================================================
// MEM
// =====================================================

double get_mem_usage()
{
  std::ifstream f("/proc/meminfo");

  if (!f.is_open())
    return -1;

  std::string key;

  long total = 0;
  long avail = 0;

  while (f >> key)
  {
    if (key == "MemTotal:")
    {
      f >> total;
    }
    else if (key == "MemAvailable:")
    {
      f >> avail;
      break;
    }
    else
    {
      f.ignore(1024, '\n');
    }
  }

  if (total == 0)
    return -1;

  return (1.0 - (double)avail / total) * 100.0;
}

// =====================================================
// NPU
// =====================================================

struct NpuLoad
{
  int core0 = -1;
  int core1 = -1;
  int core2 = -1;

  double avg = -1;
  double peak = -1;
};

NpuLoad get_npu_usage()
{
  FILE *fp =
      popen("cat /sys/kernel/debug/rknpu/load", "r");

  if (!fp)
  {
    return {-1, -1, -1, -1, -1};
  }

  char buf[256] = {0};

  if (!fgets(buf, sizeof(buf), fp))
  {
    pclose(fp);
    return {-1, -1, -1, -1, -1};
  }

  pclose(fp);

  LOGI("RAW NPU BUF=%s", buf);

  NpuLoad n;

  std::vector<int> nums;

  char *p = buf;

  while (*p)
  {
    if (isdigit(*p))
    {
      nums.push_back(strtol(p, &p, 10));
    }
    else
    {
      p++;
    }
  }

  if (nums.size() >= 3)
  {
    n.core0 = nums[0];
    n.core1 = nums[1];
    n.core2 = nums[2];

    n.avg =
        (n.core0 + n.core1 + n.core2) / 3.0;

    n.peak =
        std::max(
            {n.core0,
             n.core1,
             n.core2});
  }
  else if (nums.size() == 1)
  {
    n.core0 =
        n.core1 =
            n.core2 =
                nums[0];

    n.avg = nums[0];
    n.peak = nums[0];
  }
  else
  {
    return {-1, -1, -1, -1, -1};
  }

  return n;
}

// =====================================================
// ARGS
// =====================================================

struct Args
{
  std::string model =
      "./model/liquid_960p.rknn";

  std::string output =
      "./report.csv";

  int run_time = 60;

  std::vector<int> threads =
      {3, 4, 6, 8};
};

void usage()
{
  LOGI(
      "Usage:\n"
      "./rknn_npu_stress_test_demo "
      "[--model xxx.rknn] "
      "[--output xxx.csv] "
      "[--time 10] "
      "[--threads 1 2 4]");
}

Args parse_args(
    int argc,
    char **argv)
{
  Args args;

  for (int i = 1; i < argc; i++)
  {
    std::string k = argv[i];

    if (k == "--help")
    {
      usage();
      exit(0);
    }

    else if (k == "--model")
    {
      if (i + 1 < argc)
      {
        args.model = argv[++i];
      }
    }

    else if (k == "--output")
    {
      if (i + 1 < argc)
      {
        args.output = argv[++i];
      }
    }

    else if (k == "--time")
    {
      if (i + 1 < argc)
      {
        args.run_time =
            atoi(argv[++i]);
      }
    }

    else if (k == "--threads")
    {
      args.threads.clear();

      while (i + 1 < argc &&
             argv[i + 1][0] != '-')
      {
        args.threads.push_back(
            atoi(argv[++i]));
      }
    }
  }

  return args;
}

// =====================================================
// STATS
// =====================================================

struct Stats
{
  std::atomic<int> count{0};

  std::atomic<long long> latency{0};
};

// =====================================================
// WORKER
// =====================================================

class Worker
{
public:
  explicit Worker(
      const std::string &model_path)
  {
    load_model(model_path);

    init_rknn();
  }

  ~Worker()
  {
    stop();

    if (ctx)
    {
      rknn_destroy(ctx);
      ctx = 0;
    }
  }

  void run()
  {
    std::vector<uint8_t> input(
        input_size,
        0);

    rknn_input inputs[1];

    memset(inputs, 0, sizeof(inputs));

    inputs[0].index = 0;
    inputs[0].buf = input.data();
    inputs[0].size = input_size;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = input_fmt;

    while (running.load())
    {
      auto t1 =
          std::chrono::high_resolution_clock::now();

      int ret =
          rknn_inputs_set(
              ctx,
              1,
              inputs);

      if (ret != RKNN_SUCC)
      {
        LOGE(
            "rknn_inputs_set failed=%d",
            ret);

        continue;
      }

      ret =
          rknn_run(
              ctx,
              nullptr);

      if (ret != RKNN_SUCC)
      {
        LOGE(
            "rknn_run failed=%d",
            ret);

        continue;
      }

      auto t2 =
          std::chrono::high_resolution_clock::now();

      long long us =
          std::chrono::duration_cast<
              std::chrono::microseconds>(
              t2 - t1)
              .count();

      stats.count.fetch_add(1);

      stats.latency.fetch_add(us);
    }
  }

  void stop()
  {
    running.store(false);
  }

  Stats stats;

private:
  rknn_context ctx = 0;

  int input_size = 0;

  rknn_tensor_format input_fmt;

  std::vector<unsigned char> model_data;

  std::atomic<bool> running{true};

private:
  void load_model(
      const std::string &path)
  {
    FILE *fp =
        fopen(path.c_str(), "rb");

    if (!fp)
    {
      LOGE(
          "open model failed: %s",
          path.c_str());

      exit(-1);
    }

    fseek(fp, 0, SEEK_END);

    int size = ftell(fp);

    rewind(fp);

    model_data.resize(size);

    fread(
        model_data.data(),
        1,
        size,
        fp);

    fclose(fp);

    LOGI(
        "model size=%d",
        size);
  }

  void init_rknn()
  {
    int ret =
        rknn_init(
            &ctx,
            model_data.data(),
            model_data.size(),
            0,
            nullptr);

    if (ret != RKNN_SUCC)
    {
      LOGE(
          "rknn_init failed=%d",
          ret);

      exit(-1);
    }

    rknn_tensor_attr attr;

    memset(&attr, 0, sizeof(attr));

    attr.index = 0;

    ret =
        rknn_query(
            ctx,
            RKNN_QUERY_INPUT_ATTR,
            &attr,
            sizeof(attr));

    if (ret != RKNN_SUCC)
    {
      LOGE(
          "query input failed=%d",
          ret);

      exit(-1);
    }

    input_fmt = attr.fmt;

    input_size = 1;

    for (int i = 0; i < attr.n_dims; i++)
    {
      input_size *= attr.dims[i];
    }

    LOGI(
        "input size=%d",
        input_size);
  }
};

// =====================================================
// TEST
// =====================================================

void run_test(
    int thread_num,
    const std::string &model,
    int run_time,
    std::ofstream &csv)
{
  LOGI("==================================");
  LOGI("THREAD=%d", thread_num);

  std::vector<std::shared_ptr<Worker>>
      workers;

  for (int i = 0; i < thread_num; i++)
  {
    workers.push_back(
        std::make_shared<Worker>(
            model));
  }

  std::vector<std::thread> threads;

  for (auto &w : workers)
  {
    threads.emplace_back(
        &Worker::run,
        w);
  }

  std::this_thread::sleep_for(
      std::chrono::seconds(2));

  CpuStat prev = read_cpu();

  long long total_fps = 0;

  int last = 0;

  for (int i = 0; i < run_time; i++)
  {
    std::this_thread::sleep_for(
        std::chrono::seconds(1));

    CpuStat cur =
        read_cpu();

    double cpu =
        calc_cpu(prev, cur);

    prev = cur;

    double mem =
        get_mem_usage();

    NpuLoad npu =
        get_npu_usage();

    int total = 0;

    for (auto &w : workers)
    {
      total +=
          w->stats.count.load();
    }

    int fps =
        total - last;

    last = total;

    total_fps += fps;

    LOGI(
        "[T=%d] FPS=%d CPU=%.2f%% MEM=%.2f%% NPU=%.2f%%",
        thread_num,
        fps,
        cpu,
        mem,
        npu.avg);
  }

  for (auto &w : workers)
  {
    w->stop();
  }

  for (auto &t : threads)
  {
    t.join();
  }

  double avg_fps =
      total_fps /
      (double)run_time;

  LOGI("RESULT FPS=%.2f", avg_fps);

  csv
      << thread_num << ","
      << avg_fps
      << "\n";

  csv.flush();
}

// =====================================================
// MAIN
// =====================================================

int main(
    int argc,
    char **argv)
{
  LOGI("RKNN Benchmark Start");

  Args args =
      parse_args(argc, argv);

  LOGI("model=%s",
       args.model.c_str());

  LOGI("output=%s",
       args.output.c_str());

  LOGI("run_time=%d",
       args.run_time);

  std::ofstream csv(
      args.output);

  if (!csv.is_open())
  {
    LOGE(
        "open csv failed");

    return -1;
  }

  csv << "threads,fps\n";

  for (auto t : args.threads)
  {
    run_test(
        t,
        args.model,
        args.run_time,
        csv);
  }

  csv.close();

  LOGI("benchmark done");

  return 0;
}