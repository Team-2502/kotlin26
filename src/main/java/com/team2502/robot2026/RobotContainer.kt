// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2502.robot2026

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team2502.lib.TMJoystick
import com.team2502.robot2026.Constants.OI
import com.team2502.robot2026.subsystems.IntakeSubsystem
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain
import com.team2502.robot2026.subsystems.drive.TunerConstants
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import java.util.function.Supplier

class RobotContainer {
    // Joystick mappings
    val driverLeft: TMJoystick = TMJoystick(OI.JOYSTICK_DRIVE_LEFT)
    val driverRight: TMJoystick = TMJoystick(OI.JOYSTICK_DRIVE_RIGHT)
    val operator: TMJoystick = TMJoystick(OI.JOYSTICK_OPERATOR)

    // Subsystems
    val drivetrainSubsystem: CommandSwerveDrivetrain = TunerConstants.createDrivetrain()
    val intakeSubsystem: IntakeSubsystem = IntakeSubsystem()

    // Swerve Command Setups
    private val driveRequest: SwerveRequest.FieldCentric = SwerveRequest.FieldCentric()
        .withDeadband(OI.TRANSLATION_DEADBAND_METERS_PER_SECOND)
        .withRotationalDeadband(OI.ROTATION_DEADBAND_RADIANS_PER_SECOND)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)


    init {
        configureBindings()
    }

    private fun configureBindings() {
        // Drivetrain bindings
        drivetrainSubsystem.defaultCommand = drivetrainSubsystem.applyRequest {
            driveRequest
                .withVelocityX(-driverLeft.getX() * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND) // Drive forward with negative Y (forward)
                .withVelocityY(-driverLeft.getY() * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND) // Drive left with negative X (left)
                .withRotationalRate(-driverRight.getZ() * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
        }

        // Intake Bindings
        driverRight.trigger().whileTrue(intakeSubsystem.intake()).whileFalse(intakeSubsystem.stop())
    }

    val autonomousCommand: Command
        get() = Commands.print("No autonomous command configured")
}
