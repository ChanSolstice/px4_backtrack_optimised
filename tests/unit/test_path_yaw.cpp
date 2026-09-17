#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "px4_backtrack/domain/path_yaw.hpp"

namespace px4_backtrack::domain {
namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(PathYawTest, StraightPathFacesAlongTravelDirection) {
  const std::vector<Vec3> positions = {
      Vec3(2.0, 0.0, 1.0),
      Vec3(1.0, 0.0, 1.0),
      Vec3(0.0, 0.0, 1.0),
  };

  const auto yaws = computePathTangentYaws(positions, 0.0);

  ASSERT_EQ(yaws.size(), positions.size());
  for (double yaw : yaws) {
    EXPECT_NEAR(std::abs(yaw), kPi, 1e-12);
  }
}

TEST(PathYawTest, EndpointIgnoresShortHoverDrift) {
  const std::vector<Vec3> positions = {
      Vec3(2.0, 0.0, 1.0),
      Vec3(1.0, 0.0, 1.0),
      Vec3(0.25, 0.0, 1.0),
      Vec3(0.04, -0.08, 1.0),
      Vec3(0.0, 0.0, 1.0),
  };

  const auto yaws = computePathTangentYaws(positions, 0.0);

  ASSERT_EQ(yaws.size(), positions.size());
  EXPECT_NEAR(std::abs(yaws.back()), kPi, 1e-12);
}

TEST(PathYawTest, ShortPathStillUsesAvailableDirection) {
  const std::vector<Vec3> positions = {
      Vec3(0.10, 0.0, 1.0),
      Vec3(0.0, 0.0, 1.0),
  };

  const auto yaws = computePathTangentYaws(positions, 0.3);

  ASSERT_EQ(yaws.size(), positions.size());
  EXPECT_NEAR(std::abs(yaws.front()), kPi, 1e-12);
  EXPECT_NEAR(std::abs(yaws.back()), kPi, 1e-12);
}

}  // namespace
}  // namespace px4_backtrack::domain
