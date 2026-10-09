// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2502.robot2026

import choreo.auto.AutoRoutine
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team2502.lib.TMJoystick
import com.team2502.robot2026.Constants.OI
import com.team2502.robot2026.commands.runIntakeCommand
import com.team2502.robot2026.commands.runOuttakeCommand
import com.team2502.robot2026.commands.runUnjamCommand
import com.team2502.robot2026.commands.setTurretCommand
import com.team2502.robot2026.commands.shootCommand
import com.team2502.robot2026.subsystems.IntakeSubsystem
import com.team2502.robot2026.subsystems.ShooterSubsystem
import com.team2502.robot2026.subsystems.TurretSubsystem
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain
import com.team2502.robot2026.subsystems.drive.TunerConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import java.util.concurrent.atomic.AtomicReference
import java.util.function.DoubleSupplier
import kotlin.jvm.optionals.getOrDefault
import kotlin.math.PI

class RobotContainer {
    // Joystick mappings
    val driverLeft: TMJoystick = TMJoystick(OI.JOYSTICK_DRIVE_LEFT)
    val driverRight: TMJoystick = TMJoystick(OI.JOYSTICK_DRIVE_RIGHT)
    val operator: TMJoystick = TMJoystick(OI.JOYSTICK_OPERATOR)

    // NT Config
    val networkTablesInstance = NetworkTableInstance.getDefault();

    val poseNT = networkTablesInstance.getTable("poseTable")
    val xPub = poseNT.getDoubleTopic("xPub").publish()
    val yPub = poseNT.getDoubleTopic("yPub").publish()
    val zPub = poseNT.getDoubleTopic("zPub").publish()
    val poseArrayPub = poseNT.getDoubleArrayTopic("poseArray").publish()

    val targetingNT = networkTablesInstance.getTable("targetingTable")
    val targetNamePub = targetingNT.getStringTopic("targetNamePub").publish()
    val currentZonePub = targetingNT.getStringTopic("currentZonePub").publish()

    val debugNT = networkTablesInstance.getTable("debugTable")
    val debugSlider = debugNT.getDoubleTopic("debugSlider").getEntry(0.0)
    val debugText = debugNT.getStringTopic("debugText").publish()

    // Subsystems
    val drivetrainSubsystem: CommandSwerveDrivetrain = TunerConstants.createDrivetrain()
    val intakeSubsystem: IntakeSubsystem = IntakeSubsystem()
    val shooterSubsystem: ShooterSubsystem = ShooterSubsystem()
    val turretSubsystem: TurretSubsystem = TurretSubsystem()

    // setup for global subsystem access
    init {
        INSTANCE = this
    }

    // separate from init b/c init needs to execute first
    fun initialize() {
        configureBindings()
        turretSubsystem.initialize()
        shooterSubsystem.initialize()

        debugSlider.setDefault(0.0)
    }

    fun configureBindings() {
        // Drivetrain bindings
        drivetrainSubsystem.defaultCommand = drivetrainSubsystem.applyRequest {
            val alliance = DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue)
            val rotationRate = -driverRight.z
            // NEEDS TO RENABLE TELEOP FOR ALLIANCE TO FLIP
            val velX: Double = driverLeft.y
            val velY: Double = -driverLeft.x
            SwerveRequest.FieldCentric()
                .withDeadband(OI.TRANSLATION_DEADBAND_METERS_PER_SECOND)
                .withRotationalDeadband(OI.ROTATION_DEADBAND_RADIANS_PER_SECOND)
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                .withVelocityX(velX * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND)
                .withVelocityY(velY * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND)
                .withRotationalRate(rotationRate * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
        }

        turretSubsystem.defaultCommand = setTurretCommand { MathUtil.applyDeadband(-operator.z, 0.02) }


        // Bindings
        operator.trigger().or(driverRight.trigger()).whileTrue(runIntakeCommand())
        operator.middleThumbButton().whileTrue(runOuttakeCommand())
        operator.rightThumbButton().whileTrue(runUnjamCommand())
        operator.leftThumbButton().toggleOnTrue(shootCommand({driverLeft.slider}))
        driverLeft.leftThumbButton().onTrue(drivetrainSubsystem.runOnce { drivetrainSubsystem.seedFieldCentric() })
//        driverLeft.middleThumbButton().whileTrue(setTurretCommand(Rotation2d(0.0)))
//        driverLeft.rightThumbButton().whileTrue(setTurretFieldAngleCommand(Rotation2d(0.0)))
        // driverRight.rightThumbButton().whileTrue(VisionUpdateCommand())
    }

    // Auto
    fun update() {
        val pose = INSTANCE.getPose()

        // NT Publishing
        xPub.set(pose.x)
        yPub.set(pose.y)
        zPub.set(pose.rotation.degrees)
        poseArrayPub.set(
            doubleArrayOf(
                pose.x,
                pose.y,
                pose.rotation.degrees,
                0.0,
                0.0,
                0.0
            )
        )
    }

    // global subsystem access via companion object
    companion object {
        private val INSTANCE_CONTAINER = AtomicReference<RobotContainer>()
        var INSTANCE: RobotContainer
            set(value) {
                INSTANCE_CONTAINER.set(value)
            }
            get() {
                return INSTANCE_CONTAINER.get()
            }
    }

    fun getPose(): Pose2d {
        val ret = drivetrainSubsystem.getPose()
        return Pose2d(ret.x, ret.y, getYaw())
    }

    fun getYaw(): Rotation2d {
        return Rotation2d.fromDegrees(drivetrainSubsystem.pigeon2.yaw.valueAsDouble)
    }
}
