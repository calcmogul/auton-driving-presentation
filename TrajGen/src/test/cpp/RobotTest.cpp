// Copyright (c) 2020 FIRST. All Rights Reserved.

#include <frc/RobotBase.h>
#include <gtest/gtest.h>

#include "RenameCSVs.hpp"
#include "Robot.hpp"

// Make sure robot initializes
TEST(RobotTest, Init) {
    frc::RunHALInitialization();
    Robot robot;

    RenameCSVs("RobotTest", "./Drivetrain ");
    RenameCSVs("RobotTest", "./Flywheel ");
    RenameCSVs("RobotTest", "./Turret ");
}
