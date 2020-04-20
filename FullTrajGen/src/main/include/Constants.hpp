// Copyright (c) 2020 FIRST. All Rights Reserved.

#pragma once

#include <units/units.h>
#include <wpi/math>

namespace Constants {

namespace Robot {
// Joystick Ports
constexpr int kDriveStick1Port = 0;
constexpr int kDriveStick2Port = 1;
constexpr int kAppendageStickPort = 2;
constexpr int kAppendageStick2Port = 3;
}  // namespace Robot

namespace Drivetrain {
// Motor Ports
constexpr int kLeftMasterPort = 16;
constexpr int kLeftSlavePort = 1;
constexpr int kRightMasterPort = 14;
constexpr int kRightSlavePort = 15;

// Encoder Ports
constexpr int kLeftEncoderA = 0;
constexpr int kLeftEncoderB = 1;
constexpr int kRightEncoderA = 2;
constexpr int kRightEncoderB = 3;

// Gyro port
constexpr int kGyroPort = 1;

// Motor ports
constexpr int kLeftMotor = 1;
constexpr int kRightMotor = 2;

constexpr units::meter_t kWheelRadius = 3_in;
constexpr double kDriveGearRatio = 1.0 / 1.0;
constexpr double kDpP = (2.0 * wpi::math::pi * kWheelRadius.to<double>()) *
                        kDriveGearRatio / 2048.0;

static constexpr units::meter_t kWidth = 0.990405073902434_m;

// Robot radius
static constexpr auto rb = kWidth / 2.0;

constexpr decltype(1_V / 1_mps) kLinearV = 3.02_V / 1_mps;
constexpr decltype(1_V / 1_mps_sq) kLinearA = 0.642_V / 1_mps_sq;
constexpr decltype(1_V / 1_rad_per_s) kAngularV = 1.382_V / 1_rad_per_s;
constexpr decltype(1_V / (1_rad_per_s / 1_s)) kAngularA =
    (0.3398_V / 4.0) / (1_rad_per_s / 1_s);
constexpr decltype(1_V / (1_V / 1_mps)) kMaxV = 12_V / kLinearV;
constexpr decltype(1_V / (1_V / 1_mps_sq)) kMaxA = 12_V / kLinearA;
}  // namespace Drivetrain

constexpr auto kDt = 5_ms;
}  // namespace Constants
