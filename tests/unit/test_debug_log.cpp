#include <gtest/gtest.h>

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "px4_backtrack/infrastructure/debug_log.hpp"

namespace px4_backtrack::infrastructure {
namespace {

namespace fs = std::filesystem;

/// 每个用例一个临时根目录，结束时删除
class DebugLogTest : public ::testing::Test {
protected:
  void SetUp() override {
    base_ = fs::temp_directory_path() /
            ("debug_log_test_" + std::to_string(::getpid()) + "_" +
             ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(base_);
    fs::create_directories(base_);
  }
  void TearDown() override { fs::remove_all(base_); }

  DebugLogConfig config(bool enable) const {
    DebugLogConfig c;
    c.enable = enable;
    c.dir = base_.string();
    return c;
  }

  static std::vector<std::string> readLines(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);) {
      lines.push_back(line);
    }
    return lines;
  }

  static LogRecord stateRecord(double t, const std::string& detail = "") {
    LogRecord record;
    record.t = t;
    record.flight = 2;
    record.mission = 1;
    record.event.type = domain::EventType::State;
    record.event.from = "ReturnActive";
    record.event.to = "Exited";
    record.event.rule = "4b";
    record.event.reason = "EXTERNAL_MODE";
    record.event.detail = detail;
    return record;
  }

  fs::path base_;
};

TEST_F(DebugLogTest, DisabledCreatesNothing) {
  // S30：关闭调试日志时不建目录、不写文件
  {
    DebugLog log(config(false), "run_20261021_101500");
    EXPECT_FALSE(log.enabled());
    EXPECT_TRUE(log.runDir().empty());
    log.writeConfig(0.0, {}, "abc", "unknown");
    log.write(stateRecord(1.0));
    log.flush();
    EXPECT_FALSE(log.failed());
  }
  EXPECT_TRUE(fs::is_empty(base_));
}

TEST_F(DebugLogTest, WritesConfigFirstThenEventsAsJsonLines) {
  std::string run_dir;
  {
    DebugLog log(config(true), "run_20261021_101500");
    run_dir = log.runDir();
    EXPECT_EQ(run_dir, (base_ / "run_20261021_101500").string());
    log.writeConfig(10.0, {{"ch18_confirm_s", "1.0"}, {"debug_log_dir", "\"\""}},
                    "5479d0f", "unknown");
    log.write(stateRecord(1263.25));
    log.write(stateRecord(1263.30, "quote\" back\\slash"));
    log.flush();
    EXPECT_FALSE(log.failed());
  }

  const auto lines = readLines(run_dir + "/events.jsonl");
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0],
            "{\"t\":10.000,\"type\":\"config\",\"version\":{\"module\":\"5479d0f\","
            "\"trajectory_core\":\"unknown\"},\"params\":{\"ch18_confirm_s\":1.0,"
            "\"debug_log_dir\":\"\"}}");
  EXPECT_EQ(lines[1],
            "{\"t\":1263.250,\"flight\":2,\"mission\":1,\"type\":\"state\","
            "\"from\":\"ReturnActive\",\"to\":\"Exited\",\"rule\":\"4b\","
            "\"reason\":\"EXTERNAL_MODE\"}");
  EXPECT_NE(lines[2].find("\"detail\":\"quote\\\" back\\\\slash\""), std::string::npos);
}

TEST_F(DebugLogTest, QueueOverflowDropsAndReportsCount) {
  // 不启动写线程，队列只在 flush 时写出，结果确定
  DebugLog log(config(true), "run_overflow", 2, false);
  for (int i = 0; i < 5; ++i) {
    log.write(stateRecord(i));
  }
  log.flush();  // 写出前两行，丢弃 3 行
  log.write(stateRecord(10.0));
  log.flush();

  const auto lines = readLines(log.runDir() + "/events.jsonl");
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_NE(lines[0].find("\"t\":0.000"), std::string::npos);
  EXPECT_NE(lines[1].find("\"t\":1.000"), std::string::npos);
  EXPECT_EQ(lines[2], "{\"t\":10.000,\"type\":\"log_dropped\",\"count\":3}");
  EXPECT_NE(lines[3].find("\"t\":10.000,\"flight\""), std::string::npos);
}

TEST_F(DebugLogTest, DirectoryFailureStopsLogging) {
  // 根目录是一个普通文件：建不了目录，停止写日志，返航不受影响
  const fs::path file = base_ / "not_a_dir";
  std::ofstream(file.string()) << "x";
  DebugLogConfig c = config(true);
  c.dir = file.string();

  DebugLog log(c, "run_20261021_101500");
  EXPECT_TRUE(log.failed());
  log.write(stateRecord(1.0));
  log.flush();
  EXPECT_TRUE(log.failed());
}

TEST_F(DebugLogTest, RestartWithinSameSecondGetsNewDirectory) {
  std::string first;
  {
    DebugLog log(config(true), "run_20261021_101500");
    first = log.runDir();
  }
  DebugLog log(config(true), "run_20261021_101500");
  EXPECT_NE(log.runDir(), first);
  EXPECT_EQ(log.runDir(), (base_ / "run_20261021_101500_2").string());
}

}  // namespace
}  // namespace px4_backtrack::infrastructure
