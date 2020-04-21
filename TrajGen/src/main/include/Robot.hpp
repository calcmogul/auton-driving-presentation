// Copyright (c) 2020 FIRST. All Rights Reserved.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/Timer.h>

#include "Constants.hpp"
#include "Encoder.h"

class Robot : public frc::TimedRobot {
public:
    Robot();

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    /**
     * Returns true if feedback controllers reached the reference.
     */
    bool AtReference() const;

    frc::TrajectoryConfig MakeTrajectoryConfig() const;

    // Drivetrain encoders
    Encoder m_leftEncoder{Constants::Drivetrain::kLeftEncoderA,
                          Constants::Drivetrain::kLeftEncoderB};
    Encoder m_rightEncoder{Constants::Drivetrain::kRightEncoderA,
                           Constants::Drivetrain::kRightEncoderB};

    // Motors
    frc::PWMVictorSPX m_leftMotor{Constants::Drivetrain::kLeftMotor};
    frc::PWMVictorSPX m_rightMotor{Constants::Drivetrain::kRightMotor};

private:
    // AnalogGyro instead of ADXRS450 for ease of simulation
    frc::AnalogGyro m_gyro{Constants::Drivetrain::kGyroPort};

    // Localization
    frc::DifferentialDriveKinematics m_kinematics{
        Constants::Drivetrain::kWidth};
    frc::DifferentialDriveOdometry m_odometry{0_rad};

    // Model identified using
    // https://github.com/wpilibsuite/frc-characterization
    frc::LinearSystem<2, 2, 2> m_plant = frc::IdentifyDrivetrainSystem(
        Constants::Drivetrain::kLinearV.to<double>(),
        Constants::Drivetrain::kLinearA.to<double>(),
        Constants::Drivetrain::kAngularV.to<double>(),
        Constants::Drivetrain::kAngularA.to<double>());

    // Controllers
    frc::RamseteController m_ramsete;
    frc::LinearQuadraticRegulator<2, 2> m_lqr{
        m_plant, {0.95, 0.95}, {12.0, 12.0}, Constants::kDt};
    frc2::PIDController m_leftPID{m_lqr.K(0, 0), 0.0, 0.0, Constants::kDt};
    frc2::PIDController m_rightPID{m_lqr.K(1, 1), 0.0, 0.0, Constants::kDt};

    // Autonomous variables
    frc::Trajectory m_trajectory;
    frc2::Timer m_timer;

    // The loggers that generate the comma separated value files
    frc::CSVLogFile positionLogger{"Drivetrain Positions",
                                   "Estimated X (m)",
                                   "Estimated Y (m)",
                                   "X Ref (m)",
                                   "Y Ref (m)",
                                   "Measured Left Position (m)",
                                   "Measured Right Position (m)"};
    frc::CSVLogFile angleLogger{"Drivetrain Angles", "Measured Heading (rad)",
                                "Estimated Heading (rad)", "Heading Ref (rad)"};
    frc::CSVLogFile velocityLogger{"Drivetrain Velocities",
                                   "Estimated Left Vel (m/s)",
                                   "Estimated Right Vel (m/s)",
                                   "Left Vel Ref (m/s)", "Right Vel Ref (m/s)"};
    frc::CSVLogFile voltageLogger{"Drivetrain Voltages", "Left Voltage (V)",
                                  "Right Voltage (V)", "Battery Voltage (V)"};
};
