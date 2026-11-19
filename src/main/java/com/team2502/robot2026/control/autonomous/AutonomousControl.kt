package com.team2502.robot2026.control.autonomous

import choreo.auto.AutoFactory
import com.team2502.lib.Localization
import com.team2502.lib.Localization.awaitNewPose
import com.team2502.robot2026.control.common.FieldTarget
import com.team2502.robot2026.control.common.calculateShot
import com.team2502.robot2026.subsystems.RobotContainer
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlin.jvm.optionals.getOrNull
import kotlin.math.absoluteValue
import kotlin.time.Duration.Companion.milliseconds

class AutonomousControl(private val robotContainer: RobotContainer) {
    val pathBuilder = FollowPath.Builder(
        robotContainer.drivetrain.subsystem,
        { Localization.getCurrentPose()?.pose },
        { Localization.getCurrentPose()?.state?.Speeds },
        robotContainer.drivetrain::driveRobotRelative,
        PIDController(2.0, 0.0, 0.0),
        PIDController(20.0, 0.0, 6.25),
        PIDController(0.2, 0.0, 0.0)
    ).withTRatioBasedTranslationHandoffs(true)

    private val autoChooser = SendableChooser<String>()
    private val autoFactory = AutoFactory(
        { Localization.getCurrentPose()?.pose },
        robotContainer.drivetrain::resetPose,
        robotContainer.drivetrain::followTrajectory,
        true,
        robotContainer.drivetrain.subsystem
    )
    private val Rotation0 = Path(
        Path.Waypoint(
            Translation2d(14.0, 4.0),
            Rotation2d(0.0),
        ),
    )
    private val Rotation180 = Path(
        Path.Waypoint(
            Translation2d(14.0, 2.5),
            Rotation2d(180.0),
        ),
    )

    private val autonNT = NetworkTableInstance.getDefault().getTable("PID")
    private val kPSub = autonNT.getDoubleTopic("kP").getEntry(0.0)
    private val kISub = autonNT.getDoubleTopic("kI").getEntry(0.0)
    private val kDSub = autonNT.getDoubleTopic("kD").getEntry(0.0)
    private val errorPub = autonNT.getDoubleTopic("error").publish()


//// Builder options persist. Clear this before building any later path command.
//		robotContainer.drivetrain.pathBuilder.withPoseReset({ ignored -> })
    // TODO: Overrides withPoseReset function to not reset on new path > implement

    init {
        kPSub.setDefault(0.0)
        kISub.setDefault(0.0)
        kDSub.setDefault(0.0)
        errorPub.setDefault(0.0)
    }

    private suspend fun testPID() {
        while (true) {
            shuffleX(3.0)
            delay(1000.milliseconds)
            shuffleX(-3.0)
            delay(1000.milliseconds)
        }
    }

    private suspend fun shuffleY(delta: Double) {
        awaitNewPose()
        val startingY = Localization.getCurrentPose()?.pose?.translation?.y ?: return
        val setpoint = startingY + delta
        val controller = PIDController(kPSub.get(), kISub.get(), kDSub.get())
        try {
            var goodPoses = 0
            while (true) {
                val pose = awaitNewPose() ?: break
                val error = pose.pose.translation.y - setpoint
                errorPub.set(error)
                if (error.absoluteValue < 0.05) {
                    goodPoses++
                } else {
                    goodPoses = 0
                }
                if (goodPoses > 20) {
                    break
                }
                val gain = controller.calculate(pose.pose.translation.y, setpoint)
                robotContainer.drivetrain.driveDirection(0.0, -gain, 0.0)
            }
        } finally {
            robotContainer.drivetrain.stop()
        }
    }

    private suspend fun shuffleX(delta: Double) {
        awaitNewPose()
        val startingX = Localization.getCurrentPose()?.pose?.translation?.x ?: return
        val setpoint = startingX + delta
        // TODO: Make this + other one constants
        val controller = PIDController(16.3, kISub.get(), 1.1)
        try {
            var goodPoses = 0
            while (true) {
                val pose = awaitNewPose() ?: break
                val error = pose.pose.translation.x - setpoint
                errorPub.set(error)
                if (error.absoluteValue < 0.05) {
                    goodPoses++
                } else {
                    goodPoses = 0
                }
                if (goodPoses > 20) {
                    break
                }
                val gain = controller.calculate(pose.pose.translation.x, setpoint)
                robotContainer.drivetrain.driveDirection(-gain, 0.0, 0.0)
            }
        } finally {
            robotContainer.drivetrain.stop()
        }
    }

    private suspend fun turnTo(rotation: Rotation2d) {
        val controller = PIDController(11.1, 0.0, 0.4)
        try {
            var goodPoses = 0
            while (true) {
                awaitNewPose()
                val pose = Localization.getCurrentPose() ?: break
                val error = pose.pose.rotation.minus(rotation)
                if (error.degrees.absoluteValue < 3.0) {
                    goodPoses++
                } else {
                    goodPoses = 0
                }
                if (goodPoses > 20) {
                    break
                }
                val gain = controller.calculate(pose.pose.rotation.radians, rotation.radians)
                robotContainer.drivetrain.driveDirection(0.0, 0.0, gain)
            }
        } finally {
            robotContainer.drivetrain.stop()
        }
    }

    suspend fun run() {
        val alliance = DriverStation.getAlliance().getOrNull()
        if (alliance == null) {
            System.err.println("ERROR: Not on any alliance!")
            return
        }
        try {
            coroutineScope {
                testPID()
//                val firstPath = pathBuilder
//                    .withPoseReset({ _ -> })
//                    .build(Rotation0)
//                runCommand(firstPath)
//                val secondPath = pathBuilder
//                    .withPoseReset({ _ -> })
//                    .build(Rotation180)
//                runCommand(secondPath)
            }
        } finally {
            robotContainer.drivetrain.stop()
            robotContainer.intake.stop()
        }
    }

    private suspend fun shootAtHub(alliance: DriverStation.Alliance) {
        try {
            while (true) {
                val pose = awaitNewPose() ?: return

                val target = when (alliance) {
                    DriverStation.Alliance.Blue -> FieldTarget.BLUE_HUB
                    DriverStation.Alliance.Red -> FieldTarget.RED_HUB
                }
                val currentFlywheelSpeed = robotContainer.shooter.flywheelSpeed()
                val shot = calculateShot(pose, currentFlywheelSpeed, target)
                robotContainer.shooter.aimAndShoot(shot)
            }
        } finally {
            robotContainer.shooter.stop()
        }
    }
}
