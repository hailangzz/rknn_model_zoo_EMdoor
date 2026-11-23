
/*
 * RKNN NPU Long-time Stress Test
 *
 * Target:
 *   RK3588
 *   Linux AArch64
 *   GCC 6.3.1
 *   C++11
 *
 * Features:
 *   1. Multi-thread RKNN inference
 *   2. One RKNN context per worker
 *   3. Dynamic input/output attribute query
 *   4. CPU / Memory / NPU load monitoring
 *   5. CSV performance logging
 *   6. Runtime configuration
 *   7. Graceful Ctrl+C shutdown
 *   8. No std::filesystem dependency
 */

#include <rknn_api.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

/* ============================================================
 * Global configuration
 * ============================================================ */

static volatile sig_atomic_t g_stop = 0;

static std::mutex g_log_mutex;
static std::mutex g_csv_mutex;

static const int DEFAULT_THREADS = 3;
static const int DEFAULT_HOURS = 12;
static const int DEFAULT_INTERVAL = 5;

static std::string g_model_path = "./model/liquid_960p.rknn";
static std::string g_csv_path = "./results/npu_stress.csv";

static int g_thread_count = DEFAULT_THREADS;
static int g_duration_hours = DEFAULT_HOURS;
static int g_monitor_interval = DEFAULT_INTERVAL;
static bool g_show_help = false;

static std::chrono::steady_clock::time_point g_start_time;

/* ============================================================
 * Signal handling
 * ============================================================ */

static void signal_handler(int signal_number)
{
  (void)signal_number;
  g_stop = 1;
}

/* ============================================================
 * Logging
 * ============================================================ */

static void log_message(const std::string &level,
                        const std::string &message)
{
  std::lock_guard<std::mutex> lock(g_log_mutex);

  std::time_t now = std::time(NULL);
  struct tm tm_now;

  localtime_r(&now, &tm_now);

  char time_buffer[64];

  std::strftime(time_buffer,
                sizeof(time_buffer),
                "%Y-%m-%d %H:%M:%S",
                &tm_now);

  std::cout << "[" << time_buffer << "] "
            << "[" << level << "] "
            << message << std::endl;
}

/* ============================================================
 * POSIX filesystem utilities
 * ============================================================ */

static bool file_exists(const std::string &path)
{
  return access(path.c_str(), F_OK) == 0;
}

static bool make_directories(const std::string &path)
{
  if (path.empty())
    return false;

  std::string current;

  for (size_t i = 0; i < path.size(); ++i)
  {
    current += path[i];

    if (path[i] == '/' || i == path.size() - 1)
    {
      if (current.empty() || current == "/")
        continue;

      if (mkdir(current.c_str(), 0755) != 0 &&
          errno != EEXIST)
      {
        return false;
      }

      struct stat st;

      if (stat(current.c_str(), &st) != 0 ||
          !S_ISDIR(st.st_mode))
      {
        return false;
      }
    }
  }

  return true;
}

static bool ensure_parent_directory(const std::string &filepath)
{
  size_t pos = filepath.find_last_of('/');

  if (pos == std::string::npos)
    return true;

  if (pos == 0)
    return true;

  return make_directories(filepath.substr(0, pos));
}

/* ============================================================
 * Model loading
 * ============================================================ */

static bool load_model(const std::string &path,
                       std::vector<unsigned char> &model_data)
{
  FILE *fp = fopen(path.c_str(), "rb");

  if (fp == NULL)
  {
    log_message("ERROR", "Cannot open model: " + path);
    return false;
  }

  if (fseek(fp, 0, SEEK_END) != 0)
  {
    fclose(fp);
    log_message("ERROR", "Model fseek failed");
    return false;
  }

  long file_size = ftell(fp);

  if (file_size <= 0)
  {
    fclose(fp);
    log_message("ERROR", "Invalid model file size");
    return false;
  }

  if (fseek(fp, 0, SEEK_SET) != 0)
  {
    fclose(fp);
    log_message("ERROR", "Model rewind failed");
    return false;
  }

  model_data.resize(static_cast<size_t>(file_size));

  size_t read_size = fread(
      &model_data[0],
      1,
      static_cast<size_t>(file_size),
      fp);

  fclose(fp);

  if (read_size != static_cast<size_t>(file_size))
  {
    model_data.clear();
    log_message("ERROR", "Model read incomplete");
    return false;
  }

  log_message(
      "INFO",
      "Model loaded, size = " +
          std::to_string(file_size) + " bytes");

  return true;
}

/* ============================================================
 * CPU monitor
 * ============================================================ */

struct CpuCounters
{
  unsigned long long total;
  unsigned long long idle;

  CpuCounters()
      : total(0), idle(0)
  {
  }
};

static bool read_cpu_counters(CpuCounters &counters)
{
  std::ifstream file("/proc/stat");

  if (!file.is_open())
    return false;

  std::string line;

  if (!std::getline(file, line))
    return false;

  std::istringstream iss(line);

  std::string cpu_label;

  unsigned long long user = 0;
  unsigned long long nice = 0;
  unsigned long long system = 0;
  unsigned long long idle = 0;
  unsigned long long iowait = 0;
  unsigned long long irq = 0;
  unsigned long long softirq = 0;
  unsigned long long steal = 0;

  if (!(iss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal))
  {
    return false;
  }

  counters.idle = idle + iowait;

  counters.total =
      user + nice + system + idle +
      iowait + irq + softirq + steal;

  return true;
}

class CpuMonitor
{
public:
  CpuMonitor()
      : initialized_(false)
  {
  }

  double sample()
  {
    CpuCounters current;

    if (!read_cpu_counters(current))
      return -1.0;

    if (!initialized_)
    {
      previous_ = current;
      initialized_ = true;
      return -1.0;
    }

    unsigned long long total_delta =
        current.total - previous_.total;

    unsigned long long idle_delta =
        current.idle - previous_.idle;

    previous_ = current;

    if (total_delta == 0)
      return -1.0;

    double usage =
        100.0 *
        (double)(total_delta - idle_delta) /
        (double)total_delta;

    return std::max(0.0, std::min(100.0, usage));
  }

private:
  bool initialized_;
  CpuCounters previous_;
};

/* ============================================================
 * Memory monitor
 * ============================================================ */

struct MemoryInfo
{
  double total_mb;
  double used_mb;
  double usage_percent;

  MemoryInfo()
      : total_mb(-1.0),
        used_mb(-1.0),
        usage_percent(-1.0)
  {
  }
};

static MemoryInfo read_memory_info()
{
  MemoryInfo result;

  std::ifstream file("/proc/meminfo");

  if (!file.is_open())
    return result;

  unsigned long long mem_total = 0;
  unsigned long long mem_free = 0;
  unsigned long long mem_available = 0;
  unsigned long long buffers = 0;
  unsigned long long cached = 0;

  std::string line;

  while (std::getline(file, line))
  {
    std::istringstream iss(line);

    std::string key;
    unsigned long long value = 0;

    if (!(iss >> key >> value))
      continue;

    if (key == "MemTotal:")
      mem_total = value;
    else if (key == "MemFree:")
      mem_free = value;
    else if (key == "MemAvailable:")
      mem_available = value;
    else if (key == "Buffers:")
      buffers = value;
    else if (key == "Cached:")
      cached = value;
  }

  if (mem_total == 0)
    return result;

  unsigned long long available = mem_available;

  if (available == 0)
    available = mem_free + buffers + cached;

  if (available > mem_total)
    available = mem_total;

  unsigned long long used = mem_total - available;

  result.total_mb = mem_total / 1024.0;
  result.used_mb = used / 1024.0;
  result.usage_percent =
      100.0 * (double)used / (double)mem_total;

  return result;
}

/* ============================================================
 * NPU monitor
 *
 * Typical RK3588 debug interface:
 *   /sys/kernel/debug/rknpu/load
 *
 * The exact output format may differ between BSP versions.
 * This parser extracts percentage values from the file.
 * ============================================================ */

static double read_npu_load()
{
  const char *paths[] =
      {
          "/sys/kernel/debug/rknpu/load",
          "/sys/kernel/debug/rknpu/load0"};

  for (size_t p = 0; p < sizeof(paths) / sizeof(paths[0]); ++p)
  {
    std::ifstream file(paths[p]);

    if (!file.is_open())
      continue;

    std::string content;
    std::string line;

    while (std::getline(file, line))
    {
      content += line;
      content += " ";
    }

    double sum = 0.0;
    int count = 0;

    const char *ptr = content.c_str();

    while (*ptr)
    {
      if (*ptr >= '0' && *ptr <= '9')
      {
        char *end_ptr = NULL;
        long value = strtol(ptr, &end_ptr, 10);

        if (end_ptr != ptr)
        {
          const char *next = end_ptr;

          while (*next == ' ' || *next == '\t')
            ++next;

          if (*next == '%')
          {
            if (value >= 0 && value <= 100)
            {
              sum += (double)value;
              ++count;
            }
          }

          ptr = end_ptr;
          continue;
        }
      }

      ++ptr;
    }

    if (count > 0)
      return sum / count;
  }

  return -1.0;
}

/* ============================================================
 * Worker statistics
 * ============================================================ */

struct WorkerStats
{
  std::atomic<unsigned long long> inference_count;
  std::atomic<unsigned long long> failure_count;
  std::atomic<unsigned long long> total_latency_us;

  WorkerStats()
      : inference_count(0),
        failure_count(0),
        total_latency_us(0)
  {
  }
};

struct Worker
{
  int id;
  WorkerStats stats;
  std::thread thread;

  Worker(int worker_id)
      : id(worker_id)
  {
  }
};

/* ============================================================
 * RKNN worker
 * ============================================================ */

static void rknn_worker(
    Worker *worker,
    const std::vector<unsigned char> *model_data,
    const std::chrono::steady_clock::time_point &deadline)
{
  if (worker == NULL || model_data == NULL ||
      model_data->empty())
  {
    return;
  }

  const int worker_id = worker->id;

  log_message(
      "INFO",
      "Worker " + std::to_string(worker_id) +
          " initializing RKNN context");

  rknn_context ctx = 0;

  int ret = rknn_init(
      &ctx,
      const_cast<unsigned char *>(&(*model_data)[0]),
      static_cast<uint32_t>(model_data->size()),
      0,
      NULL);

  if (ret != RKNN_SUCC)
  {
    log_message(
        "ERROR",
        "Worker " + std::to_string(worker_id) +
            " rknn_init failed, ret=" + std::to_string(ret));

    worker->stats.failure_count.fetch_add(1);
    return;
  }

  rknn_input_output_num io_num;
  memset(&io_num, 0, sizeof(io_num));

  ret = rknn_query(
      ctx,
      RKNN_QUERY_IN_OUT_NUM,
      &io_num,
      sizeof(io_num));

  if (ret != RKNN_SUCC ||
      io_num.n_input == 0 ||
      io_num.n_output == 0)
  {
    log_message(
        "ERROR",
        "Worker " + std::to_string(worker_id) +
            " failed to query input/output count");

    worker->stats.failure_count.fetch_add(1);
    rknn_destroy(ctx);
    return;
  }

  log_message(
      "INFO",
      "Worker " + std::to_string(worker_id) +
          " input_num=" + std::to_string(io_num.n_input) +
          ", output_num=" + std::to_string(io_num.n_output));

  /*
   * Query every input tensor attribute.
   * Allocate a zero-filled buffer for each input.
   */

  std::vector<rknn_tensor_attr> input_attrs;
  std::vector<std::vector<unsigned char>> input_buffers;

  input_attrs.resize(io_num.n_input);
  input_buffers.resize(io_num.n_input);

  bool init_ok = true;

  for (uint32_t i = 0; i < io_num.n_input; ++i)
  {
    memset(&input_attrs[i], 0, sizeof(rknn_tensor_attr));

    input_attrs[i].index = i;

    ret = rknn_query(
        ctx,
        RKNN_QUERY_INPUT_ATTR,
        &input_attrs[i],
        sizeof(rknn_tensor_attr));

    if (ret != RKNN_SUCC)
    {
      log_message(
          "ERROR",
          "Worker " + std::to_string(worker_id) +
              " query input attr failed, index=" +
              std::to_string(i));

      init_ok = false;
      break;
    }

    uint32_t buffer_size = input_attrs[i].size_with_stride;

    if (buffer_size == 0)
      buffer_size = input_attrs[i].size;

    if (buffer_size == 0)
    {
      log_message(
          "ERROR",
          "Worker " + std::to_string(worker_id) +
              " invalid input buffer size");

      init_ok = false;
      break;
    }

    input_buffers[i].assign(buffer_size, 0);

    log_message(
        "INFO",
        "Worker " + std::to_string(worker_id) +
            " input[" + std::to_string(i) +
            "] size=" + std::to_string(buffer_size));
  }

  if (!init_ok)
  {
    worker->stats.failure_count.fetch_add(1);
    rknn_destroy(ctx);
    return;
  }

  /*
   * Prepare RKNN input descriptors.
   */

  std::vector<rknn_input> inputs(io_num.n_input);

  for (uint32_t i = 0; i < io_num.n_input; ++i)
  {
    memset(&inputs[i], 0, sizeof(rknn_input));

    inputs[i].index = i;
    inputs[i].buf = &input_buffers[i][0];
    inputs[i].size = static_cast<uint32_t>(
        input_buffers[i].size());

    inputs[i].type = input_attrs[i].type;
    inputs[i].fmt = input_attrs[i].fmt;
    inputs[i].pass_through = 0;
  }

  /*
   * Warm-up.
   *
   * Warm-up errors are recorded. If the model cannot run,
   * this worker exits instead of entering an endless loop.
   */

  const int warmup_count = 5;

  for (int i = 0; i < warmup_count; ++i)
  {
    if (g_stop)
      break;

    ret = rknn_inputs_set(
        ctx,
        io_num.n_input,
        &inputs[0]);

    if (ret != RKNN_SUCC)
    {
      log_message(
          "ERROR",
          "Worker " + std::to_string(worker_id) +
              " warm-up inputs_set failed, ret=" +
              std::to_string(ret));

      worker->stats.failure_count.fetch_add(1);
      init_ok = false;
      break;
    }

    ret = rknn_run(ctx, NULL);

    if (ret != RKNN_SUCC)
    {
      log_message(
          "ERROR",
          "Worker " + std::to_string(worker_id) +
              " warm-up inference failed, ret=" +
              std::to_string(ret));

      worker->stats.failure_count.fetch_add(1);
      init_ok = false;
      break;
    }
  }

  if (!init_ok || g_stop)
  {
    rknn_destroy(ctx);
    return;
  }

  log_message(
      "INFO",
      "Worker " + std::to_string(worker_id) +
          " started inference loop");

  /*
   * Main inference loop.
   */

  while (!g_stop &&
         std::chrono::steady_clock::now() < deadline)
  {
    ret = rknn_inputs_set(
        ctx,
        io_num.n_input,
        &inputs[0]);

    if (ret != RKNN_SUCC)
    {
      worker->stats.failure_count.fetch_add(1);

      log_message(
          "ERROR",
          "Worker " + std::to_string(worker_id) +
              " inputs_set failed, ret=" +
              std::to_string(ret));

      std::this_thread::sleep_for(
          std::chrono::milliseconds(100));

      continue;
    }

    const std::chrono::steady_clock::time_point t0 =
        std::chrono::steady_clock::now();

    ret = rknn_run(ctx, NULL);

    const std::chrono::steady_clock::time_point t1 =
        std::chrono::steady_clock::now();

    if (ret != RKNN_SUCC)
    {
      worker->stats.failure_count.fetch_add(1);

      log_message(
          "ERROR",
          "Worker " + std::to_string(worker_id) +
              " rknn_run failed, ret=" +
              std::to_string(ret));

      std::this_thread::sleep_for(
          std::chrono::milliseconds(100));

      continue;
    }

    /*
     * Retrieve outputs.
     * want_float=0: keep native output format.
     */

    std::vector<rknn_output> outputs(io_num.n_output);

    for (uint32_t i = 0; i < io_num.n_output; ++i)
    {
      memset(&outputs[i], 0, sizeof(rknn_output));

      outputs[i].index = i;
      outputs[i].want_float = 0;
    }

    ret = rknn_outputs_get(
        ctx,
        io_num.n_output,
        &outputs[0],
        NULL);

    if (ret != RKNN_SUCC)
    {
      worker->stats.failure_count.fetch_add(1);

      log_message(
          "ERROR",
          "Worker " + std::to_string(worker_id) +
              " outputs_get failed, ret=" +
              std::to_string(ret));

      std::this_thread::sleep_for(
          std::chrono::milliseconds(100));

      continue;
    }

    rknn_outputs_release(
        ctx,
        io_num.n_output,
        &outputs[0]);

    const std::chrono::steady_clock::time_point t2 =
        std::chrono::steady_clock::now();

    unsigned long long latency_us =
        static_cast<unsigned long long>(
            std::chrono::duration_cast<
                std::chrono::microseconds>(t2 - t0)
                .count());

    worker->stats.inference_count.fetch_add(1);
    worker->stats.total_latency_us.fetch_add(latency_us);
  }

  rknn_destroy(ctx);

  log_message(
      "INFO",
      "Worker " + std::to_string(worker_id) +
          " stopped");
}

/* ============================================================
 * CSV
 * ============================================================ */

static bool initialize_csv()
{
  if (!ensure_parent_directory(g_csv_path))
  {
    log_message(
        "ERROR",
        "Cannot create CSV parent directory: " + g_csv_path);

    return false;
  }

  FILE *fp = fopen(g_csv_path.c_str(), "w");

  if (fp == NULL)
  {
    log_message(
        "ERROR",
        "Cannot create CSV file: " + g_csv_path);

    return false;
  }

  fprintf(
      fp,
      "elapsed_seconds,"
      "cpu_usage_percent,"
      "memory_usage_percent,"
      "memory_used_mb,"
      "memory_total_mb,"
      "npu_load_percent,"
      "worker_id,"
      "inference_count,"
      "inference_fps,"
      "failure_count,"
      "average_latency_ms\n");

  fclose(fp);

  return true;
}

static void append_csv_row(
    double elapsed_seconds,
    double cpu_usage,
    const MemoryInfo &memory,
    double npu_usage,
    int worker_id,
    unsigned long long inference_count,
    double fps,
    unsigned long long failure_count,
    double average_latency_ms)
{
  std::lock_guard<std::mutex> lock(g_csv_mutex);

  FILE *fp = fopen(g_csv_path.c_str(), "a");

  if (fp == NULL)
  {
    log_message("ERROR", "Cannot append CSV");
    return;
  }

  fprintf(
      fp,
      "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%llu,%.2f,%llu,%.3f\n",
      elapsed_seconds,
      cpu_usage,
      memory.usage_percent,
      memory.used_mb,
      memory.total_mb,
      npu_usage,
      worker_id,
      inference_count,
      fps,
      failure_count,
      average_latency_ms);

  fclose(fp);
}

/* ============================================================
 * Command line
 * ============================================================ */

static void print_usage(const char *program)
{
  std::cout
      << "\nUsage:\n"
      << "  " << program
      << " --model model.rknn"
      << " [--threads 3]"
      << " [--hours 12]"
      << " [--interval 5]"
      << " [--csv ./npu_stress.csv]\n\n"

      << "Options:\n"
      << "  --model       RKNN model path (default: ./model/liquid_960p.rknn)\n"
      << "  --threads     Number of inference workers\n"
      << "  --hours       Test duration in hours\n"
      << "  --interval    Monitoring interval in seconds\n"
      << "  --csv         CSV output path (default: ./results/npu_stress.csv)\n"
      << "  --help        Show this message\n\n";
}

static bool parse_arguments(int argc, char **argv)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];

    if (arg == "--help" || arg == "-h")
    {
      g_show_help = true;
      print_usage(argv[0]);
      return false;
    }
    else if (arg == "--model" && i + 1 < argc)
    {
      g_model_path = argv[++i];
    }
    else if (arg == "--threads" && i + 1 < argc)
    {
      g_thread_count = atoi(argv[++i]);
    }
    else if (arg == "--hours" && i + 1 < argc)
    {
      g_duration_hours = atoi(argv[++i]);
    }
    else if (arg == "--interval" && i + 1 < argc)
    {
      g_monitor_interval = atoi(argv[++i]);
    }
    else if (arg == "--csv" && i + 1 < argc)
    {
      g_csv_path = argv[++i];
    }
    else
    {
      std::cerr << "Unknown or incomplete argument: "
                << arg << std::endl;

      return false;
    }
  }

  if (g_thread_count < 1 || g_thread_count > 64)
  {
    std::cerr << "Invalid thread count" << std::endl;
    return false;
  }

  if (g_duration_hours < 1 || g_duration_hours > 168)
  {
    std::cerr << "Invalid duration hours" << std::endl;
    return false;
  }

  if (g_monitor_interval < 1 || g_monitor_interval > 3600)
  {
    std::cerr << "Invalid monitor interval" << std::endl;
    return false;
  }

  return true;
}

/* ============================================================
 * Main
 * ============================================================ */

int main(int argc, char **argv)
{
  if (!parse_arguments(argc, argv))
  {
    return g_show_help ? 0 : 1;
  }

  if (!file_exists(g_model_path))
  {
    log_message("ERROR", "Model file does not exist");
    return 1;
  }

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  log_message("INFO", "======================================");
  log_message("INFO", "RKNN NPU Stress Test");
  log_message("INFO", "======================================");

  log_message("INFO", "Model: " + g_model_path);
  log_message("INFO", "Configuration: default values are used unless overridden by command-line options");
  log_message(
      "INFO",
      "Threads: " + std::to_string(g_thread_count));
  log_message(
      "INFO",
      "Duration: " + std::to_string(g_duration_hours) + " hours");
  log_message(
      "INFO",
      "Monitor interval: " +
          std::to_string(g_monitor_interval) + " seconds");
  log_message("INFO", "CSV: " + g_csv_path);

  /*
   * Load model once.
   * All workers share the immutable model buffer.
   * It remains alive until every worker exits.
   */

  std::vector<unsigned char> model_data;

  if (!load_model(g_model_path, model_data))
    return 1;

  if (!initialize_csv())
    return 1;

  /*
   * Calculate test deadline.
   */

  g_start_time = std::chrono::steady_clock::now();

  const std::chrono::steady_clock::time_point deadline =
      g_start_time +
      std::chrono::hours(g_duration_hours);

  /*
   * Create workers.
   */

  std::vector<std::unique_ptr<Worker>> workers;

  for (int i = 0; i < g_thread_count; ++i)
  {
    workers.push_back(
        std::unique_ptr<Worker>(new Worker(i)));
  }

  for (int i = 0; i < g_thread_count; ++i)
  {
    workers[i]->thread = std::thread(
        rknn_worker,
        workers[i].get(),
        &model_data,
        deadline);
  }

  log_message("INFO", "All workers launched");

  /*
   * Monitoring loop.
   */

  CpuMonitor cpu_monitor;

  std::vector<unsigned long long> previous_counts(
      g_thread_count, 0);

  std::vector<unsigned long long> previous_latency(
      g_thread_count, 0);

  while (!g_stop &&
         std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(
        std::chrono::seconds(g_monitor_interval));

    if (g_stop)
      break;

    double elapsed_seconds =
        std::chrono::duration_cast<
            std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - g_start_time)
            .count() /
        1000.0;

    double cpu_usage = cpu_monitor.sample();

    MemoryInfo memory = read_memory_info();

    double npu_usage = read_npu_load();

    log_message(
        "MONITOR",
        "Elapsed=" + std::to_string(elapsed_seconds) +
            "s CPU=" + std::to_string(cpu_usage) +
            "% MEM=" + std::to_string(memory.usage_percent) +
            "% NPU=" + std::to_string(npu_usage) + "%");

    for (int i = 0; i < g_thread_count; ++i)
    {
      unsigned long long count =
          workers[i]->stats.inference_count.load();

      unsigned long long failures =
          workers[i]->stats.failure_count.load();

      unsigned long long total_latency =
          workers[i]->stats.total_latency_us.load();

      unsigned long long delta_count =
          count - previous_counts[i];

      unsigned long long delta_latency =
          total_latency - previous_latency[i];

      double fps =
          (double)delta_count /
          (double)g_monitor_interval;

      double average_latency_ms = 0.0;

      if (delta_count > 0)
      {
        average_latency_ms =
            (double)delta_latency /
            (double)delta_count / 1000.0;
      }

      append_csv_row(
          elapsed_seconds,
          cpu_usage,
          memory,
          npu_usage,
          i,
          count,
          fps,
          failures,
          average_latency_ms);

      previous_counts[i] = count;
      previous_latency[i] = total_latency;

      log_message(
          "WORKER",
          "Worker " + std::to_string(i) +
              " total=" + std::to_string(count) +
              " FPS=" + std::to_string(fps) +
              " failures=" + std::to_string(failures) +
              " latency=" +
              std::to_string(average_latency_ms) + "ms");
    }
  }

  /*
   * Stop all workers.
   */

  g_stop = 1;

  log_message("INFO", "Stopping workers...");

  for (int i = 0; i < g_thread_count; ++i)
  {
    if (workers[i]->thread.joinable())
      workers[i]->thread.join();
  }

  /*
   * Final summary.
   */

  unsigned long long total_inferences = 0;
  unsigned long long total_failures = 0;

  for (int i = 0; i < g_thread_count; ++i)
  {
    total_inferences +=
        workers[i]->stats.inference_count.load();

    total_failures +=
        workers[i]->stats.failure_count.load();
  }

  double actual_seconds =
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - g_start_time)
          .count() /
      1000.0;

  double average_fps = 0.0;

  if (actual_seconds > 0.0)
  {
    average_fps =
        (double)total_inferences / actual_seconds;
  }

  log_message("INFO", "======================================");
  log_message("INFO", "Stress Test Finished");
  log_message(
      "INFO",
      "Actual duration: " +
          std::to_string(actual_seconds) + " seconds");
  log_message(
      "INFO",
      "Total inference count: " +
          std::to_string(total_inferences));
  log_message(
      "INFO",
      "Total failures: " +
          std::to_string(total_failures));
  log_message(
      "INFO",
      "Average FPS: " +
          std::to_string(average_fps));
  log_message("INFO", "CSV: " + g_csv_path);
  log_message("INFO", "======================================");

  return 0;
}
