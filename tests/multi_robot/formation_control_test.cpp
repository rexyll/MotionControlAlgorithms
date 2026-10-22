#include "mca/multi_robot/formation_control.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::multi_robot::FormationInput;
using mca::multi_robot::FormationOutput;
using mca::multi_robot::formation_control;

FormationInput triangle_input() {
  FormationInput input;
  input.positions.resize(3, 2);
  input.positions << 0.0, 0.0,
                     2.0, 0.0,
                     0.0, 2.0;
  input.desired_offsets.resize(3, 2);
  input.desired_offsets << 0.0, 0.0,
                           1.0, 0.0,
                           2.0, 0.0;
  input.adjacency = Eigen::MatrixXd::Ones(3, 3);
  input.adjacency.diagonal().setZero();
  input.gain = 0.5;
  input.max_speed = 10.0;
  return input;
}

TEST(FormationControl, CommandsReduceRelativeOffsetError) {
  FormationInput input = triangle_input();
  FormationOutput output;
  ASSERT_TRUE(formation_control(input, output));
  // Agent 0 is too close to agent 2 relative to the desired x offset.
  EXPECT_LT(output.velocities(0, 0), 0.0);
  // Agent 2 is too far from agent 0 in x and too high in y.
  EXPECT_LT(output.velocities(2, 1), 0.0);
  EXPECT_TRUE(output.velocities.allFinite());
}

TEST(FormationControl, EnforcesPerAgentSpeedLimit) {
  FormationInput input = triangle_input();
  input.gain = 100.0;
  input.max_speed = 0.25;
  FormationOutput output;
  ASSERT_TRUE(formation_control(input, output));
  for (Eigen::Index i = 0; i < output.velocities.rows(); ++i)
    EXPECT_LE(output.velocities.row(i).norm(), 0.25 + 1e-12);
}

TEST(FormationControl, TranslationOfOffsetsDoesNotChangeCommands) {
  FormationInput input = triangle_input();
  FormationOutput baseline;
  ASSERT_TRUE(formation_control(input, baseline));
  input.desired_offsets.rowwise() += Eigen::RowVector2d(12.0, -7.0);
  FormationOutput shifted;
  ASSERT_TRUE(formation_control(input, shifted));
  EXPECT_LT((baseline.velocities - shifted.velocities).norm(), 1e-12);
}

TEST(FormationControl, RejectsInvalidInputsTransactionally) {
  FormationInput input = triangle_input();
  FormationOutput output;
  output.velocities = Eigen::MatrixXd::Constant(1, 1, 42.0);
  input.adjacency(0, 1) = -1.0;
  std::string error;
  EXPECT_FALSE(formation_control(input, output, &error));
  EXPECT_FALSE(error.empty());
  ASSERT_EQ(output.velocities.rows(), 1);
  EXPECT_DOUBLE_EQ(output.velocities(0, 0), 42.0);
  input = triangle_input();
  input.positions(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(formation_control(input, output));
}
}  // namespace
