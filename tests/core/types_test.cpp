#include <gtest/gtest.h>

#include "mca/core/types.hpp"

TEST(CoreTypes, AngleConversion) {
  EXPECT_NEAR(mca::core::deg_to_rad(180.0), 3.14159265358979323846, 1e-15);
  EXPECT_NEAR(mca::core::rad_to_deg(3.14159265358979323846), 180.0, 1e-12);
}

TEST(CoreTypes, Status) {
  const mca::core::Status ok{};
  EXPECT_TRUE(ok.ok());
  EXPECT_FALSE(mca::core::Status{mca::core::StatusCode::InvalidArgument}.ok());
}
