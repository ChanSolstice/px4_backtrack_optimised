#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "px4_backtrack/domain/return_state_machine.hpp"

namespace px4_backtrack {
namespace infrastructure {

// ============================================================================
// 配置参数
// ============================================================================

/// DebugLog配置参数
struct DebugLogConfig {
  // 运行时参数由 ConfigLoader 从 backtrack_params.yaml 注入。
  bool enable{};    ///< 是否写调试日志
  std::string dir;  ///< 日志根目录；空为 ~/.ros/backtrack_logs，非空必须是绝对路径
  int keep_runs{};  ///< 保留的运行目录个数（含本次）
};

/// 写入 events.jsonl 的一条事件
struct LogRecord {
  double t{0.0};    ///< 时间 (s)
  int flight{0};    ///< 飞行周期编号
  int mission{0};   ///< 任务编号，没有任务时为 0
  domain::StepEvent event;
};

// ============================================================================
// DebugLog类
// ============================================================================

/// 调试日志（重构方案第 9.2 节，D29）
///
/// 每次节点启动建一个运行目录，事件一行一个写入 events.jsonl；只保留最近 keep_runs 个
/// 运行目录（按目录名排序，本次的不删除）。写盘在后台线程，
/// 控制线程只把一行文本放进有界队列；队列满时丢弃并计数，之后补写一条
/// log_dropped。建目录或写文件失败时停止写日志（failed 为真），返航不受影响。
/// 关闭时不建目录、不写文件。
class DebugLog {
public:
  /// 构造函数
  /// @param config 配置参数
  /// @param run_name 运行目录名，如 run_20261021_101500
  /// @param queue_capacity 队列容量（行）
  /// @param start_writer 是否启动后台写线程；为假时只在 flush 时写盘（单测用）
  DebugLog(const DebugLogConfig& config, const std::string& run_name,
           std::size_t queue_capacity = 4096, bool start_writer = true);

  /// 析构：写完队列后退出写线程
  ~DebugLog();

  DebugLog(const DebugLog&) = delete;
  DebugLog& operator=(const DebugLog&) = delete;

  /// 是否启用
  bool enabled() const { return config_.enable; }

  /// 本次运行目录；未启用时为空
  const std::string& runDir() const { return run_dir_; }

  /// 建目录或写文件失败（DEBUG_LOG_FAILED）
  bool failed() const;

  /// 写入 config 事件：生效参数与版本，应为第一条
  /// @param params (参数名, JSON 值文本) 列表
  void writeConfig(double t,
                   const std::vector<std::pair<std::string, std::string>>& params,
                   const std::string& module_version,
                   const std::string& core_version);

  /// 写入一条事件（不阻塞）
  void write(const LogRecord& record);

  /// 等待队列写完
  void flush();

  /// 默认日志根目录 ~/.ros/backtrack_logs
  static std::string defaultDir();

  /// 转成带引号的 JSON 字符串
  static std::string jsonString(const std::string& text);

private:
  /// 把一行放进队列；队列满时丢弃并计数
  void enqueue(std::string line, double t);

  /// 把队列中的行写入文件（调用时不持锁）
  void writeLines(std::deque<std::string>& lines);

  /// 后台写线程
  void writerLoop();

  /// 配置参数
  DebugLogConfig config_;
  std::string run_dir_;
  std::size_t capacity_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::condition_variable drained_cv_;
  std::deque<std::string> queue_;
  std::size_t dropped_{0};   ///< 尚未写出 log_dropped 的丢弃行数
  bool writing_{false};      ///< 写线程正持有一批行
  bool stop_{false};
  bool failed_{false};

  std::ofstream file_;
  std::thread writer_;
};

}  // namespace infrastructure
}  // namespace px4_backtrack
