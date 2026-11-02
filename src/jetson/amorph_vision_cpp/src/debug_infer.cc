// 离线调试工具：对单张图片跑一次实例分割，打印检测结果并可选保存可视化图。
//
// 用途：不启动 ROS、不需要相机，用来验证 engine、插件与 letterbox 掩膜映射是否正确。
//
// 用法:
//   ros2 run amorph_vision_cpp amorph_vision_cpp_debug_infer \
//       --engine resource/best.engine --image test.jpg --out annotated.jpg
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

#include "amorph_vision_cpp/types.hpp"
#include "amorph_vision_cpp/yolo_segmenter.hpp"

namespace {

struct Options {
  std::string engine;
  std::string image;
  std::string output;
  float conf = 0.25F;
  int input_width = 640;
  int input_height = 640;
};

void PrintUsage() {
  std::fprintf(stderr,
               "用法: %s --engine <path> --image <path> [--out <path>] "
               "[--conf <0-1>] [--input-size <w> <h>]\n",
               "amorph_vision_cpp_debug_infer");
}

bool ParseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&](std::string* value) {
      if (i + 1 >= argc) {
        return false;
      }
      *value = argv[++i];
      return true;
    };

    if (arg == "--engine") {
      if (!next(&options->engine)) {
        return false;
      }
    } else if (arg == "--image") {
      if (!next(&options->image)) {
        return false;
      }
    } else if (arg == "--out") {
      if (!next(&options->output)) {
        return false;
      }
    } else if (arg == "--conf") {
      std::string value;
      if (!next(&value)) {
        return false;
      }
      options->conf = std::stof(value);
    } else if (arg == "--input-size") {
      std::string width;
      std::string height;
      if (!next(&width) || !next(&height)) {
        return false;
      }
      options->input_width = std::stoi(width);
      options->input_height = std::stoi(height);
    } else if (arg == "-h" || arg == "--help") {
      PrintUsage();
      std::exit(0);
    } else {
      std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
      return false;
    }
  }
  return !options->engine.empty() && !options->image.empty();
}

/// 掩膜在检测框内的占比，用来粗判掩膜与框是否对齐。
double MaskFillRatio(const cv::Mat& mask, const amorph::vision::BoundingBox& bbox) {
  const int x1 = std::clamp(static_cast<int>(bbox.x1), 0, mask.cols - 1);
  const int y1 = std::clamp(static_cast<int>(bbox.y1), 0, mask.rows - 1);
  const int x2 = std::clamp(static_cast<int>(bbox.x2), 0, mask.cols);
  const int y2 = std::clamp(static_cast<int>(bbox.y2), 0, mask.rows);
  if (x2 <= x1 || y2 <= y1) {
    return 0.0;
  }
  const cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
  const double area = static_cast<double>(roi.width) * roi.height;
  return static_cast<double>(cv::countNonZero(mask(roi))) / area;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    PrintUsage();
    return 1;
  }

  cv::Mat image = cv::imread(options.image, cv::IMREAD_COLOR);
  if (image.empty()) {
    std::fprintf(stderr, "读取图片失败: %s\n", options.image.c_str());
    return 1;
  }
  std::printf("图片: %s (%dx%d)\n", options.image.c_str(), image.cols, image.rows);

  try {
    amorph::vision::YoloSegmenter segmenter(options.engine,
                                           cv::Size(options.input_width, options.input_height));
    std::printf("engine 加载成功: %s\n", options.engine.c_str());

    const std::vector<amorph::vision::SegmentDetection> detections = segmenter.Detect(image, options.conf);
    std::printf("检出 %zu 个目标（conf >= %.2f）\n", detections.size(), options.conf);

    for (std::size_t i = 0; i < detections.size(); ++i) {
      const auto& detection = detections[i];
      const int mask_pixels = cv::countNonZero(detection.mask);
      std::printf("  #%zu class=%d conf=%.3f box=[%.0f, %.0f, %.0f, %.0f] mask=%d px fill=%.2f\n", i,
                  detection.class_id, static_cast<double>(detection.confidence), detection.bbox.x1,
                  detection.bbox.y1, detection.bbox.x2, detection.bbox.y2, mask_pixels,
                  MaskFillRatio(detection.mask, detection.bbox));
    }

    if (!options.output.empty()) {
      cv::Mat canvas = image.clone();
      for (std::size_t i = 0; i < detections.size(); ++i) {
        const auto& detection = detections[i];
        const cv::Rect box(static_cast<int>(detection.bbox.x1), static_cast<int>(detection.bbox.y1),
                           static_cast<int>(detection.bbox.x2 - detection.bbox.x1),
                           static_cast<int>(detection.bbox.y2 - detection.bbox.y1));
        cv::rectangle(canvas, box, cv::Scalar(0, 165, 255), 2);

        // 掩膜轮廓画成绿色，便于目视确认与框对齐
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(detection.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(canvas, contours, -1, cv::Scalar(0, 255, 0), 2);

        const std::string label = cv::format("#%zu %.2f", i, static_cast<double>(detection.confidence));
        cv::putText(canvas, label, cv::Point(box.x, std::max(box.y - 6, 12)), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(255, 255, 255), 2);
      }
      if (cv::imwrite(options.output, canvas)) {
        std::printf("可视化已保存: %s\n", options.output.c_str());
      } else {
        std::fprintf(stderr, "保存可视化失败: %s\n", options.output.c_str());
      }
    }
  } catch (const std::exception& error) {
    std::fprintf(stderr, "推理失败: %s\n", error.what());
    return 1;
  }

  return 0;
}
