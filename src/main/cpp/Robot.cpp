/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <fstream>

#include <hal/cpp/fpga_clock.h>

void Robot::RobotInit()
{
  m_jsc.SetRange(0.05, 1.0);
  m_jsc.SetExponent(3.0);

  int i;
  for (i = 0; std::ifstream{m_csvName + std::to_string(i) + ".csv"}.good() && i < 25; i++);
  if (i >= 25) {
    i = 0;
    m_csvName = m_csvName + std::to_string(i) + ".csv";
    std::ofstream ofs;
    ofs.open(m_csvName, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
  } else {
    m_csvName = m_csvName + std::to_string(i) + ".csv";
  }
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
  m_pose.push_back(std::make_tuple(
    std::chrono::duration_cast<std::chrono::milliseconds>(hal::fpga_clock::now().time_since_epoch()).count(),
    pose.Translation().X(),
    pose.Translation().Y(),
    pose.Rotation().Degrees()
  ));

  if (m_pose.size() >= 100) WritePoseToCSV();
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
    m_jsc.Condition(m_driverJoystickLeft.GetY()),
    m_jsc.Condition(m_driverJoystickRight.GetY())
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
      << std::to_string(double(std::get<3>(tuple))) << "\n";
  }

  stream.close();
  m_pose.clear();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
