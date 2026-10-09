#include "hiwonder_xarm_esp32_hardware/hiwonder_system.hpp"
#include <gtest/gtest.h>

using hiwonder_xarm_esp32_hardware::JointCalibration;

TEST(JointMovementDuration, ReversedGripperClosesSlowlyAndOpensNormally)
{
  JointCalibration gripper;
  gripper.direction = -1.0;
  gripper.movement_duration_ms = 1000;
  gripper.closing_movement_duration_ms = 3000;
  EXPECT_EQ(gripper.movement_duration_for(270, 786), 3000);
  EXPECT_EQ(gripper.movement_duration_for(786, 270), 1000);
  EXPECT_EQ(gripper.movement_duration_for(528, 528), 1000);
}

TEST(JointMovementDuration, ClosingUsesRosDirectionWithEitherRawAxis)
{
  JointCalibration joint;
  joint.direction = 1.0;
  joint.movement_duration_ms = 1000;
  joint.closing_movement_duration_ms = 3000;
  EXPECT_EQ(joint.movement_duration_for(786, 270), 3000);
  EXPECT_EQ(joint.movement_duration_for(270, 786), 1000);
}

TEST(JointMovementDuration, ArmDurationIsUnchangedInBothDirections)
{
  JointCalibration arm;
  for (const double direction : {-1.0, 1.0}) {
    arm.direction = direction;
    EXPECT_EQ(arm.movement_duration_for(270, 786), 40);
    EXPECT_EQ(arm.movement_duration_for(786, 270), 40);
  }
}
