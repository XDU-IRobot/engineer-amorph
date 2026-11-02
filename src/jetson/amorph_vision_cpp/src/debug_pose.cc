// 离线调试工具：对单张图片 + 深度图跑完整位姿估计（YOLO → PCA → ICP）。
//
// 用途：不启动 ROS、不需要相机，用固定的一帧数据反复验证几何链路。
// 配合 scripts/capture_frame.py 使用（它会产出 color.png / depth.png / meta.txt）。
//
// 用法:
//   ros2 run amorph_vision_cpp amorph_vision_cpp_debug_pose \
//       --engine resource/best.engine --image /tmp/eu_frame \
//       --ply resource/energy_unit.ply --conf 0.5
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "amorph_vision_cpp/eu_model.hpp"
#include "amorph_vision_cpp/icp_refine.hpp"
#include "amorph_vision_cpp/pca_locator.hpp"
#include "amorph_vision_cpp/types.hpp"
#include "amorph_vision_cpp/yolo_segmenter.hpp"

namespace {

struct Options {
  std::string engine;
  std::string image_dir;
  std::string ply;
  float conf = 0.25F;
  int input_width = 640;
  int input_height = 640;
  /// 重复跑多少轮。采样剖析需要足够长的运行时间（单帧只有几十毫秒，样本太少）。
  int repeat = 1;
  /// 大于 1 时只打印每阶段累计耗时，不逐目标打印位姿，避免日志瓶颈。
  bool quiet = false;
  /// 并行线程数。与节点同构：1 为串行，可用于验证并行数值一致性。
  int jobs = 1;
};

/// 单个目标的位姿估计结果。先算完再顺序打印，保证输出顺序与检测顺序一致。
struct TargetResult {
  bool pca_ok = false;
  bool has_axis = false;
  amorph::vision::Pose pca_pose;
  Eigen::Vector3d axis = Eigen::Vector3d::Zero();
  amorph::vision::Pose refined;
  std::size_t scene_points = 0;
  std::size_t model_points = 0;
  double pca_ms = 0.0;
  double extract_ms = 0.0;
  double sample_ms = 0.0;
  double icp_ms = 0.0;
};

void PrintUsage() {
  std::fprintf(stderr,
               "用法: %s --engine <path> --image <dir> --ply <path> "
               "[--conf <0-1>] [--input-size <w> <h>] [--repeat <n>] [--jobs <n>] [--quiet]\n"
               "  --jobs 1 为串行，>1 为按目标并行（与节点一致）\n"
               "  目录内需要 color.png / depth.png / meta.txt\n",
               "amorph_vision_cpp_debug_pose");
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
      if (!next(&options->image_dir)) {
        return false;
      }
    } else if (arg == "--ply") {
      if (!next(&options->ply)) {
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
    } else if (arg == "--repeat") {
      std::string value;
      if (!next(&value)) {
        return false;
      }
      options->repeat = std::max(1, std::stoi(value));
    } else if (arg == "--jobs") {
      std::string value;
      if (!next(&value)) {
        return false;
      }
      options->jobs = std::max(1, std::stoi(value));
    } else if (arg == "--quiet") {
      options->quiet = true;
    } else if (arg == "-h" || arg == "--help") {
      PrintUsage();
      std::exit(0);
    } else {
      std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
      return false;
    }
  }
  return !options->engine.empty() && !options->image_dir.empty() && !options->ply.empty();
}

/// 读 key=value 形式的元数据文件（由 scripts/capture_frame.py 生成）。
bool ReadMeta(const std::string& path, amorph::vision::Intrinsics* intrinsics) {
  std::ifstream file(path);
  if (!file.is_open()) {
    std::fprintf(stderr, "读取 meta 失败: %s\n", path.c_str());
    return false;
  }
  std::map<std::string, double> values;
  std::string line;
  while (std::getline(file, line)) {
    const auto pos = line.find('=');
    if (pos == std::string::npos) {
      continue;
    }
    try {
      values[line.substr(0, pos)] = std::stod(line.substr(pos + 1));
    } catch (const std::exception&) {
      // 非数值字段（如 frame_id）直接忽略
    }
  }
  const auto get = [&](const std::string& key, double fallback) {
    const auto it = values.find(key);
    return it == values.end() ? fallback : it->second;
  };
  intrinsics->fx = get("fx", 0.0);
  intrinsics->fy = get("fy", 0.0);
  intrinsics->cx = get("cx", 0.0);
  intrinsics->cy = get("cy", 0.0);
  return intrinsics->fx > 0.0 && intrinsics->fy > 0.0;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    PrintUsage();
    return 1;
  }

  const cv::Mat color = cv::imread(options.image_dir + "/color.png", cv::IMREAD_COLOR);
  if (color.empty()) {
    std::fprintf(stderr, "读取彩色图失败: %s/color.png\n", options.image_dir.c_str());
    return 1;
  }
  // 16 位深度图必须用 IMREAD_UNCHANGED，否则会被缩放到 8 位
  const cv::Mat depth_raw = cv::imread(options.image_dir + "/depth.png", cv::IMREAD_UNCHANGED);
  if (depth_raw.empty()) {
    std::fprintf(stderr, "读取深度图失败: %s/depth.png\n", options.image_dir.c_str());
    return 1;
  }

  amorph::vision::Intrinsics intrinsics;
  if (!ReadMeta(options.image_dir + "/meta.txt", &intrinsics)) {
    return 1;
  }

  cv::Mat depth_mm;
  if (depth_raw.type() == CV_16U) {
    depth_raw.convertTo(depth_mm, CV_32F);
  } else if (depth_raw.type() == CV_32F) {
    depth_mm = depth_raw.clone();
  } else {
    std::fprintf(stderr, "深度图类型不支持: %d（需要 16 位或 float32）\n", depth_raw.type());
    return 1;
  }
  if (depth_mm.size() != color.size()) {
    std::fprintf(stderr, "彩色图与深度图尺寸不一致: %dx%d vs %dx%d\n", color.cols, color.rows, depth_mm.cols,
                 depth_mm.rows);
    return 1;
  }

  const int valid = cv::countNonZero(depth_mm > 0.0F);
  std::printf("图片: %dx%d, 深度有效像素 %d (%.1f%%)\n", color.cols, color.rows, valid,
              100.0 * valid / (color.cols * color.rows));
  std::printf("内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n", intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);

  try {
    amorph::vision::YoloSegmenter segmenter(options.engine, cv::Size(options.input_width, options.input_height));
    amorph::vision::EuModel model(options.ply);
    amorph::vision::PcaLocator locator(intrinsics);
    std::printf("模型采样点: %zu, 最大半径 %.4fm\n", model.size(), model.max_extent());

    using Clock = std::chrono::steady_clock;
    const auto to_ms = [](const auto& d) { return std::chrono::duration<double, std::milli>(d).count(); };

    // 累计计时（--repeat 时用于求平均值）
    double total_yolo = 0.0;
    double total_pca = 0.0;
    double total_extract = 0.0;
    double total_sample = 0.0;
    double total_icp = 0.0;
    // 并行区间的墙钟耗时。上面那些是各目标耗时的累加，对并行不敏感（不会变小），
    // 只有墙钟才反映真实加速。
    double total_estimate_wall = 0.0;
    std::size_t total_objects = 0;

    for (int round = 0; round < options.repeat; ++round) {
      const auto yolo_start = Clock::now();
      const auto detections = segmenter.Detect(color, options.conf);
      const auto yolo_end = Clock::now();
      total_yolo += to_ms(yolo_end - yolo_start);

      if (round == 0 && !options.quiet) {
        std::printf("检出 %zu 个目标\n\n", detections.size());
      }

      // --jobs > 1 时按目标并行，与 eu_locator_node::OnFrame 采用同一套写法：
      // 各目标纯函数式地算到自己的槽位，再顺序汇总，输出顺序与检测顺序一致。
      std::vector<TargetResult> results(detections.size());
      const auto target_count = static_cast<std::ptrdiff_t>(detections.size());
      const int threads = static_cast<int>(std::min<std::ptrdiff_t>(std::max<std::ptrdiff_t>(target_count, 1),
                                                                    static_cast<std::ptrdiff_t>(options.jobs)));

      const auto estimate_start = Clock::now();
#pragma omp parallel for schedule(dynamic, 1) if (threads > 1) num_threads(threads)
      for (std::ptrdiff_t k = 0; k < target_count; ++k) {
        const std::size_t i = static_cast<std::size_t>(k);
        const auto& detection = detections[i];
        TargetResult& result = results[i];

        const auto pca_start = Clock::now();
        const auto pca = locator.ProcessMask(detection.mask, detection.bbox, depth_mm);
        result.pca_ms = to_ms(Clock::now() - pca_start);
        if (!pca.has_value()) {
          continue;
        }
        result.pca_ok = true;
        result.pca_pose = pca->pose;
        if (pca->axis_direction.has_value()) {
          result.has_axis = true;
          result.axis = *pca->axis_direction;
        }

        // 分阶段计时，定位性能瓶颈
        auto t0 = Clock::now();
        const auto target = amorph::vision::ExtractMaskedPoints(depth_mm, pca->binary_mask, intrinsics);
        auto t1 = Clock::now();
        const auto source = model.SampleVisible(result.pca_pose);
        auto t2 = Clock::now();
        result.refined = amorph::vision::RefinePose(*source, *target, result.pca_pose);
        auto t3 = Clock::now();

        result.scene_points = target->size();
        result.model_points = source->size();
        result.extract_ms = to_ms(t1 - t0);
        result.sample_ms = to_ms(t2 - t1);
        result.icp_ms = to_ms(t3 - t2);
      }
      total_estimate_wall += to_ms(Clock::now() - estimate_start);

      const bool verbose = !options.quiet && round == 0;
      for (std::size_t i = 0; i < results.size(); ++i) {
        const TargetResult& result = results[i];
        const auto& detection = detections[i];
        total_pca += result.pca_ms;

        if (verbose) {
          std::printf("=== #%zu conf=%.3f box=[%.0f, %.0f, %.0f, %.0f]\n", i, static_cast<double>(detection.confidence),
                      detection.bbox.x1, detection.bbox.y1, detection.bbox.x2, detection.bbox.y2);
        }
        if (!result.pca_ok) {
          if (verbose) {
            std::printf("  PCA 失败（掩膜/深度不足）\n\n");
          }
          continue;
        }
        ++total_objects;
        total_extract += result.extract_ms;
        total_sample += result.sample_ms;
        total_icp += result.icp_ms;

        if (verbose) {
          const auto& pca_pose = result.pca_pose;
          std::printf("  PCA   pos=[%.4f, %.4f, %.4f] |t|=%.4fm\n", pca_pose.position.x(), pca_pose.position.y(),
                      pca_pose.position.z(), pca_pose.position.norm());
          if (result.has_axis) {
            std::printf("        主轴=[%.4f, %.4f, %.4f]\n", result.axis.x(), result.axis.y(), result.axis.z());
          } else {
            std::printf("        主轴: 无（点数不足）\n");
          }

          const auto& refined = result.refined;
          const double delta = (refined.position - pca_pose.position).norm();
          std::printf("  场景点=%zu 可见模型点=%zu ICP 位移=%.4fm\n", result.scene_points, result.model_points, delta);
          std::printf("  耗时: extract=%.2fms sample=%.2fms icp=%.2fms\n", result.extract_ms, result.sample_ms,
                      result.icp_ms);
          std::printf("  ICP   pos=[%.4f, %.4f, %.4f] |t|=%.4fm\n", refined.position.x(), refined.position.y(),
                      refined.position.z(), refined.position.norm());
          std::printf("  quat  [x=%.4f, y=%.4f, z=%.4f, w=%.4f]\n\n", refined.orientation.x(), refined.orientation.y(),
                      refined.orientation.z(), refined.orientation.w());
        }
      }
    }

    if (options.repeat > 1) {
      const double rounds = static_cast<double>(options.repeat);
      const double objects = static_cast<double>(std::max<std::size_t>(total_objects, 1));
      std::printf("\n--- %d 轮平均 ---\n", options.repeat);
      std::printf("YOLO      : %8.3f ms/帧\n", total_yolo / rounds);
      std::printf("PCA       : %8.3f ms/目标\n", total_pca / objects);
      std::printf("extract   : %8.3f ms/目标\n", total_extract / objects);
      std::printf("sample    : %8.3f ms/目标\n", total_sample / objects);
      std::printf("ICP       : %8.3f ms/目标\n", total_icp / objects);
      std::printf("估计墙钟  : %8.3f ms/帧（jobs=%d, %zu 目标）\n", total_estimate_wall / rounds, options.jobs,
                  total_objects / static_cast<std::size_t>(options.repeat));
      std::printf("合计      : %8.3f ms/帧（各阶段累加）\n",
                  total_yolo / rounds + (total_pca + total_extract + total_sample + total_icp) / rounds);
    }
  } catch (const std::exception& error) {
    std::fprintf(stderr, "失败: %s\n", error.what());
    return 1;
  }

  return 0;
}
