#ifndef DEBUG_NV21_SAVER_HPP
#define DEBUG_NV21_SAVER_HPP

#include <string>
#include "types.h"
#include "hand_detect_interface.h"
#include "image_drawing.h"
#include <mutex>
#include "postprocess.h"



class DebugNv21Saver {
public:
    explicit DebugNv21Saver(const std::string& save_dir);

    // 将 NV21 保存为 JPG
    void saveRgbFrame(const AndroidImageNV21& img);
    void saveRgbFrameDetect(image_buffer_t* img,std::vector<object_detect_result>& results);

private:
    bool ensureDirectory(const std::string& dir);
    std::string generateFileName();
    cv::Mat plotDetectBoxsMat(cv::Mat & image_mat,const std::vector<object_detect_result>& boxes);
    void drawPalmBoxes(cv::Mat& image, const std::vector<object_detect_result>& boxes);
private:
    std::string save_dir_;
    int frame_count_;
    std::mutex save_mutex_; 
};


#endif
