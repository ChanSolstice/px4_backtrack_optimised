#include <gtest/gtest.h>
#include "px4_backtrack/domain/recorded_route.hpp"
using namespace px4_backtrack::domain;
TEST(RecordedRouteTest, ThreeDimensionalDeviationAndBrakingBudget) {
  RecordedRoute route;
  ASSERT_TRUE(route.build({RecordPoint(Vec3(0, 0, 0), 0, 0), RecordPoint(Vec3(2, 0, 0), 0, 1)}));
  ASSERT_TRUE(route.update(Vec3(1.5, 0, 0.15)));
  EXPECT_NEAR(route.distance(), 0.15, 1e-9);
  EXPECT_NEAR(route.predictedDeviation(Vec3(1.5, 0, 0.15), Vec3(0, 0, 0.3), 0.1, 0.5, 0.05), 0.32,
              1e-9);
  EXPECT_NEAR(route.predictedDeviation(Vec3(1.5, 0, 0.15), Vec3(0.3, 0, 0), 0.1, 0.5, 0.05), 0.20,
              1e-9);
}
TEST(RecordedRouteTest, LocalWindowCannotJumpToLaterCrossing) {
  RecordedRoute route;
  ASSERT_TRUE(route.build({RecordPoint(Vec3(0, 0, 0), 0, 0), RecordPoint(Vec3(1, 0, 0), 0, 1),
                           RecordPoint(Vec3(1, 2, 0), 0, 2), RecordPoint(Vec3(0, 2, 0), 0, 3),
                           RecordPoint(Vec3(0, 0, 0), 0, 4)}));
  ASSERT_TRUE(route.update(Vec3(0.5, 0, 0)));
  EXPECT_GT(route.distance(), 0.4); // 起点只能沿首次出发支路投影
}
