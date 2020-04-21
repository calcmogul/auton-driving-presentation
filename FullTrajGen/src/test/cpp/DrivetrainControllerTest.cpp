// Copyright (c) 2020 FIRST. All Rights Reserved.

#include <cmath>

#include <Eigen/Core>
#include <frc/RobotBase.h>
#include <frc/StateSpaceUtil.h>
#include <frc/system/RungeKutta.h>
#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <simulation/AnalogGyroSim.h>
#include <simulation/EncoderSim.h>
#include <simulation/PWMSim.h>
#include <simulation/RoboRioSim.h>
#include <units/units.h>
#include <wpi/timestamp.h>

#include "Constants.hpp"
#include "RenameCSVs.hpp"
#include "Robot.hpp"

static constexpr bool kIdealModel = true;

class State {
public:
    static constexpr int kX = 0;
    static constexpr int kY = 1;
    static constexpr int kHeading = 2;
    static constexpr int kLeftVelocity = 3;
    static constexpr int kRightVelocity = 4;
    static constexpr int kLeftPosition = 5;
    static constexpr int kRightPosition = 6;
    static constexpr int kLeftVoltageError = 7;
    static constexpr int kRightVoltageError = 8;
    static constexpr int kAngularVelocityError = 9;
};

class Input {
public:
    static constexpr int kLeftVoltage = 0;
    static constexpr int kRightVoltage = 1;
};

class LocalOutput {
public:
    static constexpr int kHeading = 0;
    static constexpr int kLeftPosition = 1;
    static constexpr int kRightPosition = 2;
};

Eigen::Matrix<double, 10, 1> Dynamics(const Eigen::Matrix<double, 10, 1>& x,
                                      const Eigen::Matrix<double, 2, 1>& u) {
    auto plant = frc::IdentifyDrivetrainSystem(
        Constants::Drivetrain::kLinearV.to<double>(),
        Constants::Drivetrain::kLinearA.to<double>(),
        Constants::Drivetrain::kAngularV.to<double>(),
        Constants::Drivetrain::kAngularA.to<double>());

    Eigen::Matrix<double, 4, 2> B;
    B.block<2, 2>(0, 0) = plant.B();
    B.block<2, 2>(2, 0).setZero();
    Eigen::Matrix<double, 4, 7> A;
    A.block<2, 2>(0, 0) = plant.A();

    A.block<2, 2>(2, 0).setIdentity();
    A.block<4, 2>(0, 2).setZero();
    A.block<4, 2>(0, 4) = B;
    A.block<4, 1>(0, 6) << 0, 0, 1, -1;

    double v = (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0;

    Eigen::Matrix<double, 10, 1> result;
    result(0) = v * std::cos(x(State::kHeading));
    result(1) = v * std::sin(x(State::kHeading));
    result(2) = ((x(State::kRightVelocity) - x(State::kLeftVelocity)) /
                 (2.0 * Constants::Drivetrain::rb))
                    .to<double>();
    result.block<4, 1>(3, 0) = A * x.block<7, 1>(3, 0) + B * u;
    result.block<3, 1>(7, 0).setZero();
    return result;
}

Eigen::Matrix<double, 3, 1> LocalMeasurementModel(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);

    Eigen::Matrix<double, 3, 1> y;
    y << x(State::kHeading), x(State::kLeftPosition), x(State::kRightPosition);
    return y;
}

static auto currentTime = 0_s;

TEST(DrivetrainControllerTest, ReachesReference) {
    using Constants::kDt;

    frc::sim::RoboRioSim roboRIO{0};

    // Robot mock objects
    frc::RunHALInitialization();
    Robot robot;
    frc::sim::AnalogGyroSim simGyro{Constants::Drivetrain::kGyroPort};
    frc::sim::EncoderSim simLeftEncoder{0};
    frc::sim::EncoderSim simRightEncoder{1};
    frc::sim::PWMSim simLeftMotor{Constants::Drivetrain::kLeftMotor};
    frc::sim::PWMSim simRightMotor{Constants::Drivetrain::kRightMotor};

    // Set up simulation timestamp
    wpi::SetNowImpl([]() -> uint64_t {
        return units::microsecond_t{currentTime}.to<uint64_t>();
    });

    robot.AutonomousInit();

    Eigen::Matrix<double, 10, 1> x = Eigen::Matrix<double, 10, 1>::Zero();

    Eigen::Matrix<double, 2, 1> u = Eigen::Matrix<double, 2, 1>::Zero();

    currentTime = 0_s;
    while (currentTime < 15_s) {
        auto dt = kDt;

        // Add scheduling jitter
        if constexpr (!kIdealModel) {
            dt += units::second_t{frc::MakeWhiteNoiseVector(0.001)(0)};
        }

        Eigen::Matrix<double, 3, 1> y =
            LocalMeasurementModel(x, Eigen::Matrix<double, 2, 1>::Zero());

        // Add measurement noise
        if constexpr (!kIdealModel) {
            y += frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005);
        }

        // Set sensors based on simulation output vector
        auto heading = units::radian_t{y(LocalOutput::kHeading)};
        simGyro.SetAngle(units::degree_t{heading}.to<double>());
        robot.m_leftEncoder.SetCount(y(LocalOutput::kLeftPosition) /
                                     Constants::Drivetrain::kDpP);
        robot.m_leftEncoder.SetRate(x(State::kLeftVelocity) /
                                    Constants::Drivetrain::kDpP);
        robot.m_rightEncoder.SetCount(y(LocalOutput::kRightPosition) /
                                      Constants::Drivetrain::kDpP);
        robot.m_rightEncoder.SetRate(x(State::kRightVelocity) /
                                     Constants::Drivetrain::kDpP);

        robot.AutonomousPeriodic();
        currentTime += dt;

        // Get control inputs for sim
        u = frc::MakeMatrix<2, 1>(robot.m_leftMotor.Get() * 12.0,
                                  robot.m_rightMotor.Get() * 12.0);

        // Account for battery voltage drop due to current draw
        if constexpr (!kIdealModel) {
            constexpr auto Vbat = 12_V;
            constexpr auto Rbat = 0.03_Ohm;
            constexpr auto r = 0.0746125_m;  // Wheel radius

            constexpr auto motors = frc::DCMotor::MiniCIM(3);
            units::ampere_t loadIleft = motors.Current(
                units::meters_per_second_t{x(State::kLeftVelocity)} / r * 1_rad,
                units::volt_t{u(Input::kLeftVoltage)});
            units::ampere_t loadIright = motors.Current(
                units::meters_per_second_t{x(State::kRightVelocity)} / r *
                    1_rad,
                units::volt_t{u(Input::kRightVoltage)});
            units::volt_t vLoaded = Vbat - loadIleft * Rbat - loadIright * Rbat;
            double dsVoltage =
                vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0);
            roboRIO.SetVInVoltage(dsVoltage);

            u *= dsVoltage / 12.0;
        }

        x = frc::RungeKutta(Dynamics, x, u, dt);
    }

    RenameCSVs("DrivetrainControllerTest", "./Drivetrain ");

    EXPECT_TRUE(robot.AtReference());
}
