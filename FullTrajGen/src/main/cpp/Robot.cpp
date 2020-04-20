// Copyright (c) 2020 FIRST. All Rights Reserved.

#include "Robot.hpp"

#include <frc/RobotController.h>
#include <frc/trajectory/constraint/DrivetrainVelocitySystemConstraint.h>

Robot::Robot() {
    m_leftEncoder.SetDistancePerPulse(Constants::Drivetrain::kDpP);
    m_rightEncoder.SetDistancePerPulse(Constants::Drivetrain::kDpP);
}

void Robot::AutonomousInit() {
    m_odometry.ResetPosition(frc::Pose2d{0_m, 0_m, 0_rad}, 0_rad);
    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, 0_rad), {},
        frc::Pose2d(4.8768_m, 2.7432_m, 0_rad), MakeTrajectoryConfig());
    m_timer.Reset();
    m_timer.Start();
}

void Robot::AutonomousPeriodic() {
    m_odometry.Update(frc::Rotation2d{units::degree_t{m_gyro.GetAngle()}},
                      units::meter_t{m_leftEncoder.GetDistance()},
                      units::meter_t{m_rightEncoder.GetDistance()});
    frc::Trajectory::State ref = m_trajectory.Sample(m_timer.Get());

    auto [vlRef, vrRef] = m_kinematics.ToWheelSpeeds(
        {ref.velocity, 0_mps, ref.velocity * ref.curvature});
    positionLogger.Log(
        m_timer.Get(), m_odometry.GetPose().Translation().X().to<double>(),
        m_odometry.GetPose().Translation().Y().to<double>(),
        ref.pose.Translation().X().to<double>(),
        ref.pose.Translation().Y().to<double>(), m_leftEncoder.GetDistance(),
        m_rightEncoder.GetDistance());

    angleLogger.Log(m_timer.Get(), m_gyro.GetAngle(),
                    m_odometry.GetPose().Rotation().Radians().to<double>(),
                    ref.pose.Rotation().Radians().to<double>());
    velocityLogger.Log(m_timer.Get(), m_leftEncoder.GetRate(),
                       m_rightEncoder.GetRate(), vlRef.to<double>(),
                       vrRef.to<double>());
    voltageLogger.Log(m_timer.Get(), m_leftMotor.Get() * 12.0,
                      m_rightMotor.Get() * 12.0,
                      frc::RobotController::GetInputVoltage());

    auto chassisSpeeds = m_ramsete.Calculate(m_odometry.GetPose(), ref);
    auto [vl, vr] = m_kinematics.ToWheelSpeeds(chassisSpeeds);

    m_leftMotor.Set(
        m_leftPID.Calculate(m_leftEncoder.GetRate(), vl.to<double>()));
    m_rightMotor.Set(
        m_rightPID.Calculate(m_rightEncoder.GetRate(), vr.to<double>()));
}

bool Robot::AtReference() const { return m_ramsete.AtReference(); }

frc::TrajectoryConfig Robot::MakeTrajectoryConfig() const {
    frc::TrajectoryConfig config{Constants::Drivetrain::kMaxV,
                                 Constants::Drivetrain::kMaxA - 16.5_mps_sq};

    auto plant = frc::IdentifyDrivetrainSystem(
        Constants::Drivetrain::kLinearV.to<double>(),
        Constants::Drivetrain::kLinearA.to<double>(),
        Constants::Drivetrain::kAngularV.to<double>(),
        Constants::Drivetrain::kAngularA.to<double>());
    frc::DrivetrainVelocitySystemConstraint systemConstraint{
        plant, Constants::Drivetrain::kWidth, 8_V};
    config.AddConstraint(systemConstraint);

    return config;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
