// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2502.robot2026

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team2502.robot2026.Constants.Localization.LIMELIGHT_FRONT_NAME
import com.team2502.robot2026.Constants.Localization.LIMELIGHT_SIDE_NAME
import com.team2502.robot2026.Constants.OI
import com.team2502.robot2026.commands.shootCommand
import com.team2502.robot2026.subsystems.TargetingSubsystem
import com.team2502.robot2026.subsystems.VisionSubsystem
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import java.util.concurrent.atomic.AtomicReference
import kotlin.jvm.optionals.getOrDefault

class Robot : CoroutineRobot() {
    private var autonomousCommand: Command? = null
    private val robotContainer = RobotContainer()
    private var teleopJob: Job? = null

    init {
        robotContainer.initialize()
    }

    override fun robotPeriodic() {
        robotContainer.update()
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        autonomousCommand = robotContainer.getAuto()
        CommandScheduler.getInstance().schedule(autonomousCommand)
    }

    override fun autonomousPeriodic() {
    }

    override fun autonomousExit() {}

    override fun teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand!!.cancel()
        }
        teleopJob = launchRobot {
            while (isActive) {
                val alliance = DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue)
                var velX: Double
                var velY: Double
                val rotationRate = -RobotContainer.INSTANCE.driverRight.z
                // NEEDS TO RENABLE TELEOP FOR ALLIANCE TO FLIP
                velX = RobotContainer.INSTANCE.driverLeft.y
                velY = -RobotContainer.INSTANCE.driverLeft.x
                RobotContainer.INSTANCE.drivetrainSubsystem.setControl(
                    SwerveRequest.FieldCentric()
                        .withDeadband(OI.TRANSLATION_DEADBAND_METERS_PER_SECOND)
                        .withRotationalDeadband(OI.ROTATION_DEADBAND_RADIANS_PER_SECOND)
                        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withVelocityX(velX * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND)
                        .withVelocityY(velY * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND)
                        .withRotationalRate(rotationRate * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)

                )
                delay(20)
            }
        }
    }

    override fun teleopPeriodic() {

    }

    override fun teleopExit() {
        teleopJob?.cancel()
        teleopJob = null
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}

    override fun testExit() {}
}
