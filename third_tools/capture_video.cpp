#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 打开默认摄像头
    cv::VideoCapture cap("/dev/video0");

    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera!" << std::endl;
        return -1;
    }

    // 获取视频的帧宽度和高度
    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    // 设置视频编码，使用 XVID 编码，保存为 .avi 格式
    cv::VideoWriter writer;
    writer.open("output.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 20.0, cv::Size(frame_width, frame_height));

    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open video writer!" << std::endl;
        return -1;
    }

    cv::Mat frame;

    // 循环捕获摄像头视频并保存
    while (true) {
        // 从摄像头读取一帧
        cap >> frame;

        // 如果读取失败，跳出循环
        if (frame.empty()) {
            std::cerr << "Error: Failed to capture image!" << std::endl;
            break;
        }

        // 写入当前帧到视频文件
        writer.write(frame);

        // 显示当前帧
        cv::imshow("Camera", frame);

        // 按 'q' 键退出
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // 释放资源
    cap.release();
    writer.release();
    cv::destroyAllWindows();

    return 0;
}
