#include <cmath>  
#include <algorithm>
#include <chrono>
#include <iostream> 

class HandDetectStateController{

  public:
    // 构造函数
    HandDetectStateController(int max_frame_threshold,float max_detect_duration_s,float block_duration_s);
    // 更新函数，每帧调用
    bool update(bool detected);

  private:
      int max_frame_threshold_=1;               // 连续帧阈值
      int identification_number_;   // 判断手部出现的帧数阈值
      int counter_;                 // 当前累计帧数
      bool event_state_;            // 当前平滑后的状态
      float last_time_;             // 上次发送的时间
      float time_delay_;            // 时间延迟

      // ===== 新增 =====
    std::chrono::steady_clock::time_point detect_start_time_;

    bool in_block_state_ = false;  // 是否处于 3 秒屏蔽
    std::chrono::steady_clock::time_point block_start_time_;

    float max_detect_duration_s_ = 180.0f;  // 180 秒触发屏蔽
    float block_duration_s_ = 3.0f;         // 屏蔽持续 3 秒

};

