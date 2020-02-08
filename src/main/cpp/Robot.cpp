/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>
#include <fstream>
#include <cstdio>

#include <hal/cpp/fpga_clock.h>

void Robot::RobotInit()
{
  m_jscLeft.SetRange(kDefaultOutputOffset, 1.0);
  m_jscLeft.SetDeadband(kDefaultDeadband);
  m_jscLeft.SetExponent(kDefaultExponent);

  m_jscRight.SetRange(kDefaultOutputOffset, 1.0);
  m_jscRight.SetDeadband(kDefaultDeadband);
  m_jscRight.SetExponent(kDefaultExponent);

  frc::SmartDashboard::PutBoolean("Enable Logging", m_logData);
  frc::SmartDashboard::PutString("Driver Name", m_driverName);

  frc::SmartDashboard::PutBoolean("Update Conditioning", false);
  frc::SmartDashboard::PutNumber("Left Output Offset", kDefaultOutputOffset);
  frc::SmartDashboard::PutNumber("Left Deadband", kDefaultDeadband);
  frc::SmartDashboard::PutNumber("Left Exponent", kDefaultExponent);
  frc::SmartDashboard::PutNumber("Right Output Offset", kDefaultOutputOffset);
  frc::SmartDashboard::PutNumber("Right Deadband", kDefaultDeadband);
  frc::SmartDashboard::PutNumber("Right Exponent", kDefaultExponent);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc::Pose2d pose = m_drivetrain.UpdateOdometry();

  bool logData = frc::SmartDashboard::GetBoolean("Enable Logging", false);
  if (logData && !m_logData) {
    // Log Data rising edge begin logging data
    // Get driver name and build csv file from it
    m_driverName = frc::SmartDashboard::GetString("Driver Name", "NoName");
    m_csvName = m_driverName + "_pose_";
    int i;
    for (i = 0; std::ifstream{m_csvName + std::to_string(i) + ".csv"}.good() && i < kMaxFiles; i++);
    if (i >= kMaxFiles) {
      i = 0;
      std::ofstream ofs;
      ofs.open(m_csvName + std::to_string(i) + ".csv", std::ofstream::out | std::ofstream::trunc);
      ofs.close();
    }
    m_csvName = m_csvName + std::to_string(i) + ".csv";
    if (std::ifstream{m_csvName + std::to_string(i+1) + ".csv"}.good()) {
      remove((m_csvName + std::to_string(i+1) + ".csv").c_str());
    }
  } else if (logData && m_logData) {
    // Continue logging data
    auto inputs = m_drivetrain.GetInputs();
    m_pose.push_back(std::make_tuple(
      std::chrono::duration_cast<std::chrono::milliseconds>(hal::fpga_clock::now().time_since_epoch()).count(),
      pose.Translation().X(),
      pose.Translation().Y(),
      pose.Rotation().Degrees(),
      std::get<0>(inputs),
      std::get<1>(inputs)
    ));

    if (m_pose.size() >= 100) WritePoseToCSV();
  } else if (!logData && m_logData) {
    // Log data falling edge, stop logging data
    WritePoseToCSV();
  }

  if (frc::SmartDashboard::GetBoolean("Update Conditioning", false)) {
    m_jscLeft.SetRange(frc::SmartDashboard::PutNumber("Left Output Offset", kDefaultOutputOffset), 1.0);
    m_jscLeft.SetDeadband(frc::SmartDashboard::PutNumber("Left Deadband", kDefaultDeadband));
    m_jscLeft.SetExponent(frc::SmartDashboard::PutNumber("Left Exponent", kDefaultExponent));
    m_jscLeft.SetRange(frc::SmartDashboard::PutNumber("Right Output Offset", kDefaultOutputOffset), 1.0);
    m_jscLeft.SetDeadband(frc::SmartDashboard::PutNumber("Right Deadband", kDefaultDeadband));
    m_jscLeft.SetExponent(frc::SmartDashboard::PutNumber("Right Exponent", kDefaultExponent));
  }

  m_logData = logData;
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  m_drivetrain.SetSpeeds(
    m_jscLeft.Condition(m_driverJoystickLeft.GetY()),
    m_jscRight.Condition(m_driverJoystickRight.GetY())
  );
}

void Robot::TestPeriodic() {}

void Robot::WritePoseToCSV() {
  std::fstream stream;
  stream.open(m_csvName, std::fstream::app | std::fstream::out | std::fstream::in);
  if (!stream.is_open()) {
    std::cout << "Failed to write data to CSV file" << std::endl;
  }
  if (!m_headerWritten) {
    stream << kCSVHeader << std::endl;
    m_headerWritten = true;
  }

  for (auto& tuple : m_pose) {
    stream << std::to_string(std::get<0>(tuple)) << ","
      << std::to_string(double(std::get<1>(tuple))) << ","
      << std::to_string(double(std::get<2>(tuple))) << ","
      << std::to_string(double(std::get<3>(tuple))) << ","
      << std::to_string(double(std::get<4>(tuple))) << "\n"
      << std::to_string(double(std::get<5>(tuple))) << "\n";
  }

  stream.close();
  m_pose.clear();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
