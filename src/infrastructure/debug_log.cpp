#include "px4_backtrack/infrastructure/debug_log.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace px4_backtrack {
namespace infrastructure {

namespace fs = std::filesystem;

namespace {

void appendField(std::ostringstream& out, const char* key,
                 const std::string& value) {
  if (!value.empty()) {
    out << ",\"" << key << "\":" << DebugLog::jsonString(value);
  }
}

std::string formatTime(double t) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << t;
  return out.str();
}

std::string formatRecord(const LogRecord& record) {
  const domain::StepEvent& event = record.event;
  std::ostringstream out;
  out << "{\"t\":" << formatTime(record.t) << ",\"flight\":" << record.flight
      << ",\"mission\":" << record.mission << ",\"type\":"
      << DebugLog::jsonString(domain::eventTypeName(event.type));
  appendField(out, "name", event.name);
  appendField(out, "from", event.from);
  appendField(out, "to", event.to);
  appendField(out, "rule", event.rule);
  appendField(out, "reason", event.reason);
  appendField(out, "detail", event.detail);
  out << "}";
  return out.str();
}

}  // namespace

DebugLog::DebugLog(const DebugLogConfig& config, const std::string& run_name,
                   std::size_t queue_capacity, bool start_writer)
  : config_(config),
    capacity_(queue_capacity)
{
  if (!config_.enable) {
    return;
  }

  const std::string base = config_.dir.empty() ? defaultDir() : config_.dir;
  std::string dir = base + "/" + run_name;
  // 同一秒内重启时不与上一次的目录混在一起
  std::error_code ec;
  for (int suffix = 2; fs::exists(dir, ec); ++suffix) {
    dir = base + "/" + run_name + "_" + std::to_string(suffix);
  }
  fs::create_directories(dir, ec);
  if (ec) {
    failed_ = true;
    return;
  }
  run_dir_ = dir;
  file_.open(dir + "/events.jsonl", std::ios::out | std::ios::app);
  if (!file_) {
    failed_ = true;
    return;
  }
  if (start_writer) {
    writer_ = std::thread(&DebugLog::writerLoop, this);
  }
}

DebugLog::~DebugLog() {
  if (writer_.joinable()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_all();
    writer_.join();
  } else {
    flush();
  }
}

bool DebugLog::failed() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return failed_;
}

void DebugLog::writeConfig(
    double t, const std::vector<std::pair<std::string, std::string>>& params,
    const std::string& module_version, const std::string& core_version) {
  std::ostringstream out;
  out << "{\"t\":" << formatTime(t) << ",\"type\":\"config\",\"version\":{\"module\":"
      << jsonString(module_version) << ",\"trajectory_core\":"
      << jsonString(core_version) << "},\"params\":{";
  for (std::size_t i = 0; i < params.size(); ++i) {
    out << (i ? "," : "") << jsonString(params[i].first) << ":" << params[i].second;
  }
  out << "}}";
  enqueue(out.str(), t);
}

void DebugLog::write(const LogRecord& record) {
  if (!config_.enable) {
    return;
  }
  enqueue(formatRecord(record), record.t);
}

void DebugLog::flush() {
  if (writer_.joinable()) {
    std::unique_lock<std::mutex> lock(mutex_);
    drained_cv_.wait(lock, [this] { return queue_.empty() && !writing_; });
    return;
  }
  std::deque<std::string> lines;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lines.swap(queue_);
  }
  writeLines(lines);
}

std::string DebugLog::defaultDir() {
  const char* home = std::getenv("HOME");
  return std::string(home ? home : "/tmp") + "/.ros/backtrack_logs";
}

std::string DebugLog::jsonString(const std::string& text) {
  std::string out = "\"";
  for (const char ch : text) {
    switch (ch) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", ch);
          out += buf;
        } else {
          out += ch;
        }
    }
  }
  return out + "\"";
}

void DebugLog::enqueue(std::string line, double t) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // run_dir_ 只在构造时设置；文件只由写线程访问
    if (failed_ || run_dir_.empty()) {
      return;
    }
    // 先补写丢弃计数，再写本行；放不下两行时继续丢弃
    const std::size_t needed = dropped_ > 0 ? 2 : 1;
    if (queue_.size() + needed > capacity_) {
      ++dropped_;
      return;
    }
    if (dropped_ > 0) {
      queue_.push_back("{\"t\":" + formatTime(t) +
                       ",\"type\":\"log_dropped\",\"count\":" +
                       std::to_string(dropped_) + "}");
      dropped_ = 0;
    }
    queue_.push_back(std::move(line));
  }
  cv_.notify_one();
}

void DebugLog::writeLines(std::deque<std::string>& lines) {
  if (lines.empty()) {
    return;
  }
  for (const auto& line : lines) {
    file_ << line << '\n';
  }
  file_.flush();
  if (!file_) {
    std::lock_guard<std::mutex> lock(mutex_);
    failed_ = true;
    queue_.clear();
  }
  lines.clear();
}

void DebugLog::writerLoop() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
    if (queue_.empty()) {
      break;  // stop_ 且已写完
    }
    std::deque<std::string> lines;
    lines.swap(queue_);
    writing_ = true;
    lock.unlock();
    writeLines(lines);
    lock.lock();
    writing_ = false;
    drained_cv_.notify_all();
  }
}

}  // namespace infrastructure
}  // namespace px4_backtrack
