// 返航轨迹生成耗时基准：合成出航路径 → RecordingBuffer → TrajectorySession::generate。
// 用于评估同步规划会阻塞控制循环多久（重构方案问题 B2）。
// 用法：gen_bench <trajectory_config.yaml> [录制分钟数 ...]，默认 1 5 10 30。
#include "px4_backtrack/domain/recording_buffer.hpp"
#include "px4_backtrack/domain/trajectory_session.hpp"

#include <trajectory_core/trajectory_core.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

namespace {

using px4_backtrack::domain::RecordingBuffer;
using px4_backtrack::domain::RecordPoint;
using px4_backtrack::domain::TrajectorySession;
using px4_backtrack::domain::Vec3;

constexpr double kOdomHz = 30.0;                  // 里程计频率
constexpr double kRecordHz = 15.0;                // 与 backtrack_params.yaml 的 record_max_hz 一致
constexpr double kSpeedMps = 0.8;                 // 巡航速度
constexpr double kLegS = 5.0;                     // 每段持续时间
constexpr double kMaxTurnRad = 1.2;               // 每段随机转向上限，约 69°
constexpr double kHoverProbability = 1.0 / 6.0;   // 悬停段占比
constexpr double kAltitudeM = 5.0;
constexpr unsigned kSeed = 42;                    // 固定种子，结果可复现

std::vector<RecordPoint> synthesizeRecording(double minutes) {
  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> turn(-kMaxTurnRad, kMaxTurnRad);
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  RecordingBuffer buffer(kRecordHz);
  Vec3 position(0.0, 0.0, kAltitudeM);
  double heading = 0.0;
  bool hovering = false;
  double leg_end_s = 0.0;
  const long ticks = std::lround(minutes * 60.0 * kOdomHz);
  for (long i = 0; i < ticks; ++i) {
    const double t = static_cast<double>(i) / kOdomHz;
    if (t >= leg_end_s) {
      hovering = unit(rng) < kHoverProbability;
      if (!hovering) {
        heading += turn(rng);
      }
      leg_end_s = t + kLegS;
    }
    if (!hovering) {
      position.x() += kSpeedMps / kOdomHz * std::cos(heading);
      position.y() += kSpeedMps / kOdomHz * std::sin(heading);
    }
    buffer.addPoint(position, heading, t);
  }
  return buffer.getPointsCopy();
}

struct Result {
  double minutes;
  size_t points;
  bool ok;
  double elapsed_ms;
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <trajectory_config.yaml> [minutes ...]\n", argv[0]);
    return 2;
  }
  const auto config = trajectory_core::TrajectoryConfig::loadFromYaml(argv[1]);

  std::vector<double> durations;
  for (int i = 2; i < argc; ++i) {
    durations.push_back(std::stod(argv[i]));
  }
  if (durations.empty()) {
    durations = {1.0, 5.0, 10.0, 30.0};
  }

  std::vector<Result> results;
  for (double minutes : durations) {
    const auto points = synthesizeRecording(minutes);
    TrajectorySession session;
    const auto start = std::chrono::steady_clock::now();
    const bool ok = session.generate(points, config);
    const auto end = std::chrono::steady_clock::now();
    results.push_back({minutes, points.size(), ok,
                       std::chrono::duration<double, std::milli>(end - start).count()});
  }

  // 生成过程中的 ROS 日志会打印抽稀后点数；表格放在最后，避免与日志交错
  std::printf("\n| 录制时长 (min) | 录制点数 | 生成结果 | 生成耗时 (ms) |\n");
  std::printf("| --- | --- | --- | --- |\n");
  for (const auto& r : results) {
    std::printf("| %g | %zu | %s | %.0f |\n", r.minutes, r.points, r.ok ? "成功" : "失败",
                r.elapsed_ms);
  }
  return 0;
}
