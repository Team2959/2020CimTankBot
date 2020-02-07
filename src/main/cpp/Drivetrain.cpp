/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Drivetrain.h"

Drivetrain::Drivetrain() {
    m_gyro.Reset();
    m_leftEncoder.SetDistancePerPulse(2 * wpi::math::pi * kWheelRadius / kEncoderResolution);
    m_rightEncoder.SetDistancePerPulse(2 * wpi::math::pi * kWheelRadius / kEncoderResolution);

    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

frc::Rotation2d Drivetrain::GetAngle() {
    return frc::Rotation2d(units::degree_t(m_gyro.GetFusedHeading()));
}

const frc::Pose2d& Drivetrain::UpdateOdometry() {
    return m_odometry.Update(GetAngle(), units::meter_t(m_leftEncoder.GetDistance()), units::meter_t(m_rightEncoder.GetDistance()));
}

frc::Pose2d Drivetrain::GetPose() {
    return m_odometry.GetPose();
}

void Drivetrain::SetSpeeds(double leftSpeed, double rightSpeed) {
    m_tankDrive.TankDrive(leftSpeed, rightSpeed);
}

