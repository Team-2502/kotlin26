// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2502.robot2026

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team2502.lib.TMJoystick
import com.team2502.robot2026.Constants.OI
import com.team2502.robot2026.Constants.Weights.GYRO_EMA_WEIGHT
import com.team2502.robot2026.auto.Autos
import com.team2502.robot2026.commands.runIntakeCommand
import com.team2502.robot2026.commands.runOuttakeCommand
import com.team2502.robot2026.commands.runUnjamCommand
import com.team2502.robot2026.commands.setTurretCommand
import com.team2502.robot2026.commands.setTurretFieldAngleCommand
import com.team2502.robot2026.commands.shootCommand
import com.team2502.robot2026.subsystems.IntakeSubsystem
import com.team2502.robot2026.subsystems.ShooterSubsystem
import com.team2502.robot2026.subsystems.TargetingSubsystem
import com.team2502.robot2026.subsystems.TurretSubsystem
import com.team2502.robot2026.subsystems.VisionSubsystem
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain
import com.team2502.robot2026.subsystems.drive.TunerConstants
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import java.util.concurrent.atomic.AtomicReference
import kotlin.jvm.optionals.getOrDefault

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
    val targetingSubsystem: TargetingSubsystem = TargetingSubsystem(this) //TODO: this is bad lol

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
            var velX: Double
            var velY: Double
            val rotationRate = -driverRight.z
            if (alliance == DriverStation.Alliance.Red) {
                velX = -driverLeft.y
                velY = driverLeft.x
            } else {
                velX = driverLeft.y
                velY = -driverLeft.x
            }
            SwerveRequest.FieldCentric()
                .withDeadband(OI.TRANSLATION_DEADBAND_METERS_PER_SECOND)
                .withRotationalDeadband(OI.ROTATION_DEADBAND_RADIANS_PER_SECOND)
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                .withVelocityX(velX * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND)
                .withVelocityY(velY * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND)
                .withRotationalRate(rotationRate * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
        }

        // Intake Bindings
        driverLeft.trigger().whileTrue(runIntakeCommand())
        driverRight.leftThumbButton().whileTrue(runOuttakeCommand())
        driverRight.middleThumbButton().whileTrue(runUnjamCommand())
        driverRight.trigger().whileTrue(shootCommand())
        driverLeft.middleThumbButton().whileTrue(setTurretCommand(Rotation2d(0.0)))
        driverLeft.rightThumbButton().whileTrue(setTurretFieldAngleCommand(Rotation2d(0.0)))
        // driverRight.rightThumbButton().whileTrue(VisionUpdateCommand())
    }

    // Auto
    private val autos = Autos(drivetrainSubsystem)

    fun update() {
        targetingSubsystem.update()

        val pose = INSTANCE.getPose()

        xPub.set(pose.x)
        yPub.set(pose.y)
        zPub.set(pose.rotation.degrees)
        targetNamePub.set(targetingSubsystem.currentTarget.name)
        currentZonePub.set(targetingSubsystem.currentZone.name)
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
        debugText.set(getYaw().toString())
    }

    val autonomousCommand: Command
        get() = autos.path().cmd()

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

    private fun getYaw(): Rotation2d {
        return Rotation2d.fromDegrees(drivetrainSubsystem.pigeon2.yaw.valueAsDouble) + VisionSubsystem.pigeonZero
    }
}
