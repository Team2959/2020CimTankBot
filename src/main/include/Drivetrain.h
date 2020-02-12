/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <tuple>

#include <frc/Encoder.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Drive/DifferentialDrive.h>
#include <frc/controller/PIDController.h>

#include <wpi/math>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ahrs.h>

class Drivetrain {
private:
  static constexpr units::meter_t kTrackWidth = 0.2794_m * 2;
  static constexpr double kWheelRadius = 0.0762; // meters
  static constexpr double kEncoderResolution = 360.0;

  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_left1 {5};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_left2 {6};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_right1 {7};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_right2 {8};

  frc::SpeedControllerGroup m_left { m_left1, m_left2 };
  frc::SpeedControllerGroup m_right { m_right1, m_right2 };

  frc2::PIDController m_leftPIDController{1.0, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{1.0, 0.0, 0.0};

  frc::Encoder m_leftEncoder{0, 1};
  frc::Encoder m_rightEncoder{2, 3};
  AHRS m_gyro{SPI::Port::kMXP};

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{GetAngle()};

  frc::DifferentialDrive m_tankDrive { m_left, m_right };

  double m_leftSpeed = 0;
  double m_rightSpeed = 0;

public:
  Drivetrain();
  frc::Rotation2d GetAngle();
  const frc::Pose2d& UpdateOdometry();
  frc::Pose2d GetPose();
  std::tuple<double, double> GetSpeeds();
  void SetSpeeds(double leftSpeed, double rightSpeed);
  std::tuple<double, double> GetInputs();
};
