// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2502.robot2026

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team2502.lib.TMJoystick
import com.team2502.robot2026.Constants.OI
import com.team2502.robot2026.commands.runIntakeCommand
import com.team2502.robot2026.commands.runOuttakeCommand
import com.team2502.robot2026.commands.runUnjamCommand
import com.team2502.robot2026.subsystems.IntakeSubsystem
import com.team2502.robot2026.subsystems.ShooterSubsystem
import com.team2502.robot2026.subsystems.TurretSubsystem
import com.team2502.robot2026.subsystems.VisionSubsystem
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain
import com.team2502.robot2026.subsystems.drive.TunerConstants
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import java.util.concurrent.atomic.AtomicReference

class RobotContainer {
    // Joystick mappings
    val driverLeft: TMJoystick = TMJoystick(OI.JOYSTICK_DRIVE_LEFT)
    val driverRight: TMJoystick = TMJoystick(OI.JOYSTICK_DRIVE_RIGHT)
    val operator: TMJoystick = TMJoystick(OI.JOYSTICK_OPERATOR)

    // Subsystems
    val drivetrainSubsystem: CommandSwerveDrivetrain = TunerConstants.createDrivetrain()
    val intakeSubsystem: IntakeSubsystem = IntakeSubsystem()
    val shooterSubsystem: ShooterSubsystem = ShooterSubsystem()
    val turretSubsystem: TurretSubsystem = TurretSubsystem()

    // Swerve Command Setups
    private val driveRequest: SwerveRequest.FieldCentric = SwerveRequest.FieldCentric()
        .withDeadband(OI.TRANSLATION_DEADBAND_METERS_PER_SECOND)
        .withRotationalDeadband(OI.ROTATION_DEADBAND_RADIANS_PER_SECOND)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)


    // setup for global subsystem access
    init {
        INSTANCE = this
    }

    // separate from init b/c init needs to execute first
    fun initialize() {
        configureBindings()

    }

    private fun configureBindings() {
        // Drivetrain bindings
        drivetrainSubsystem.defaultCommand = drivetrainSubsystem.applyRequest {
            driveRequest
                .withVelocityX(-driverLeft.x * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND) // Drive forward with negative Y (forward)
                .withVelocityY(-driverLeft.y * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND) // Drive left with negative X (left)
                .withRotationalRate(-driverRight.z * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
        }

        // Intake Bindings
        driverRight.trigger().whileTrue(runIntakeCommand())
        driverRight.leftThumbButton().whileTrue(runOuttakeCommand())
        driverRight.middleThumbButton().whileTrue(runUnjamCommand())
        // driverRight.rightThumbButton().whileTrue(VisionUpdateCommand())
    }

    val autonomousCommand: Command
        get() = Commands.print("No autonomous command configured")

    // global subsystem access via companion object
    companion object {
        private val INSTANCE_CONTAINER = AtomicReference<RobotContainer>()
        var INSTANCE: RobotContainer
            set(value) { INSTANCE_CONTAINER.set(value) }
            get() { return INSTANCE_CONTAINER.get() }
    }
}
