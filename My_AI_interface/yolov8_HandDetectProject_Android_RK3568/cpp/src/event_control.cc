#include "event_control.h"
#include <algorithm>
#include <cmath>

using Clock = std::chrono::steady_clock;

HandDetectStateController::HandDetectStateController(int max_frame_threshold,float max_detect_duration_s,float block_duration_s) {
    this->max_frame_threshold_ = max_frame_threshold;
    this->identification_number_ = static_cast<int>(std::ceil(max_frame_threshold / 2.0));
    this->max_detect_duration_s_ = max_detect_duration_s;
    this->block_duration_s_ = block_duration_s;

    this->counter_ = 0;
    this->event_state_ = false;

    detect_start_time_ = Clock::time_point::min();
    block_start_time_ = Clock::time_point::min();
    in_block_state_ = false;
}

bool HandDetectStateController::update(bool detected) {

    auto now = Clock::now();

    // ---------- 屏蔽状态优先 ----------
    if (in_block_state_) {
        float block_elapsed = std::chrono::duration<float>(now - block_start_time_).count();
        if (block_elapsed >= block_duration_s_) {
            // 屏蔽结束，重置计时
            in_block_state_ = false;
            detect_start_time_ = Clock::time_point::min();
        } else {
            // 屏蔽期间，强制 false
            event_state_ = false;
            return event_state_;
        }
    }

    // ---------- 原有帧平滑逻辑 ----------
    if (detected) {
        counter_ = std::min(counter_ + 1, max_frame_threshold_);
    } else {
        counter_ = std::max(counter_ - 1, 0);
    }

    

    bool new_event_state = (counter_ >= identification_number_);

    // ---------- 计时和屏蔽逻辑 ----------
    if (new_event_state) {
        if (detect_start_time_ == Clock::time_point::min()) {
            detect_start_time_ = now; // 首次进入计时
        }

        float duration = std::chrono::duration<float>(now - detect_start_time_).count();

        if (duration >= max_detect_duration_s_) {
            // 超过 180 秒，触发屏蔽
            in_block_state_ = true;
            block_start_time_ = now;
            event_state_ = false;
            detect_start_time_ = Clock::time_point::min(); // 重置计时
        } else {
            event_state_ = true; // 持续时间小于 180 秒，正常输出 true
        }

    } else {
        // 未检测到目标，清空计时
        detect_start_time_ = Clock::time_point::min();
        event_state_ = false;
    }

    return event_state_;
}


// bool HandDetectStateController::update(bool detected) {
//     std::cout << "enter HandDetectStateController::update\n";

//     auto now = Clock::now();

//     // ---------- 屏蔽状态优先 ----------
//     if (in_block_state_) {
//         float block_elapsed = std::chrono::duration<float>(now - block_start_time_).count();
//         std::cout << "[BlockState] in_block_state_: true, block_elapsed = " << block_elapsed << "s\n";

//         if (block_elapsed >= block_duration_s_) {
//             // 屏蔽结束，重置计时
//             in_block_state_ = false;
//             detect_start_time_ = Clock::time_point::min();
//             std::cout << "[BlockState] Block ended. in_block_state_ reset to false.\n";
//         } else {
//             // 屏蔽期间，强制 false
//             event_state_ = false;
//             std::cout << "[BlockState] Block active. event_state_ = false\n";
//             return event_state_;
//         }
//     }

//     // ---------- 原有帧平滑逻辑 ----------
//     int prev_counter = counter_;
//     if (detected) {
//         counter_ = std::min(counter_ + 1, max_frame_threshold_);
//     } else {
//         counter_ = std::max(counter_ - 1, 0);
//     }
//     std::cout << "[Counter] detected = " << detected 
//               << ", prev_counter = " << prev_counter 
//               << ", counter_ = " << counter_ << "\n";

//     bool new_event_state = (counter_ >= identification_number_);
//     std::cout << "[EventCheck] new_event_state = " << new_event_state << "\n";

//     // ---------- 计时和屏蔽逻辑 ----------
//     if (new_event_state) {
//         if (detect_start_time_ == Clock::time_point::min()) {
//             detect_start_time_ = now; // 首次进入计时
//             std::cout << "[DetectTimer] Started timing.\n";
//         }

//         float duration = std::chrono::duration<float>(now - detect_start_time_).count();
//         std::cout << "[DetectTimer] duration = " << duration << "s\n";

//         if (duration >= max_detect_duration_s_) {
//             // 超过 180 秒，触发屏蔽
//             in_block_state_ = true;
//             block_start_time_ = now;
//             event_state_ = false;
//             detect_start_time_ = Clock::time_point::min(); // 重置计时
//             std::cout << "[BlockTrigger] Max detect duration reached. Trigger block.\n";
//         } else {
//             event_state_ = true; // 持续时间小于 max_detect_duration_s_，正常输出 true
//             std::cout << "[EventState] event_state_ = true\n";
//         }

//     } else {
//         // 未检测到目标，清空计时
//         detect_start_time_ = Clock::time_point::min();
//         event_state_ = false;
//         std::cout << "[EventState] No detection. event_state_ = false\n";
//     }

//     return event_state_;
// }
