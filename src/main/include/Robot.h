/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <Conditioning.h>

class Robot : public frc::TimedRobot
{
private:
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_left1 {5};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_left2 {6};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_right1 {7};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_right2 {8};

  frc::SpeedControllerGroup m_left { m_left1, m_left2 };
  frc::SpeedControllerGroup m_right { m_right1, m_right2 };

  frc::DifferentialDrive m_tankDrive { m_left, m_right };

  frc::Joystick m_driverJoystickLeft {0};
  frc::Joystick m_driverJoystickRight {1};

  cwtech::UniformConditioning m_jsc;

public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
};
