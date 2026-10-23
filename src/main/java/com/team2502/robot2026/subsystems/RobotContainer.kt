// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2502.robot2026.subsystems

import com.team2502.lib.TMJoystick
import com.team2502.robot2026.Constants.OI
import com.team2502.lib.CoroutineRobotContainer

class RobotContainer : CoroutineRobotContainer() {
    // Joystick mappings
    val driverLeftJoystick = TMJoystick(OI.JOYSTICK_DRIVE_LEFT)
    val driverRightJoystick = TMJoystick(OI.JOYSTICK_DRIVE_RIGHT)
    val operatorJoystick = TMJoystick(OI.JOYSTICK_OPERATOR)

    // Subsystems
    val drivetrain = registerSubsystem(DrivetrainSubsystem())
    val shooter = registerSubsystem(ShooterSubsystem())
    val intake = registerSubsystem(IntakeSubsystem())
}
