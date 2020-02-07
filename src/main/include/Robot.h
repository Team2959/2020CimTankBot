/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <vector>
#include <tuple>

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>

#include <Drivetrain.h>

#include <Conditioning.h>

class Robot : public frc::TimedRobot
{
private:
  frc::Joystick m_driverJoystickLeft {0};
  frc::Joystick m_driverJoystickRight {1};

  cwtech::UniformConditioning m_jsc;
  const int kMaxFiles = 5;
  const std::string kCSVHeader = "Time,Pose2d::X, Pose2d::Y, Pose2d::Rot";
  std::string m_csvName = "/home/lvuser/pose_";
  bool m_headerWritten = false;
  std::vector<std::tuple<int64_t, units::meter_t, units::meter_t, units::degree_t>> m_pose;

  Drivetrain m_drivetrain;

  void WritePoseToCSV();
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
};
