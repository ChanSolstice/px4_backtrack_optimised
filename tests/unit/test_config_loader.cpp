#include <gtest/gtest.h>

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "px4_backtrack/infrastructure/config_loader.hpp"

namespace px4_backtrack::infrastructure {
namespace {

TEST(ConfigLoaderTest, ProductionTrajectoryConfigIsValid) {
  EXPECT_NO_THROW(
      ConfigLoader::loadTrajectoryConfig(BACKTRACK_CONFIG_DIR "/trajectory_config.yaml"));
}

TEST(ConfigLoaderTest, InvalidTrajectoryConfigIsRejectedAtStartup) {
  // 收尾速度上限超过全局速度上限：校验失败，节点启动时退出（第 7.5 节，R-88）
  const auto path = std::filesystem::temp_directory_path() /
                    ("invalid_trajectory_config_" + std::to_string(::getpid()) + ".yaml");
  std::ofstream(path) << "v_max: 0.5\nv_end_max: 1.0\n";
  EXPECT_THROW(ConfigLoader::loadTrajectoryConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

}  // namespace
}  // namespace px4_backtrack::infrastructure
