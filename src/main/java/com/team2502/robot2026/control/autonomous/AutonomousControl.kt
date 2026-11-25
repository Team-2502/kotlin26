package com.team2502.robot2026.control.autonomous

import com.team2502.lib.Localization
import com.team2502.lib.Localization.awaitNewPose
import com.team2502.lib.PathGPT
import com.team2502.robot2026.Constants.Auto.ROTATION_KD
import com.team2502.robot2026.Constants.Auto.ROTATION_KI
import com.team2502.robot2026.Constants.Auto.ROTATION_KP
import com.team2502.robot2026.Constants.Auto.TRANSLATION_KD
import com.team2502.robot2026.Constants.Auto.TRANSLATION_KI
import com.team2502.robot2026.Constants.Auto.TRANSLATION_KP
import com.team2502.robot2026.Constants.Field.HALF_FIELD_WIDTH_METERS
import com.team2502.robot2026.control.common.ShooterControl
import com.team2502.robot2026.control.common.Zone
import com.team2502.robot2026.control.common.awaitZone
import com.team2502.robot2026.subsystems.RobotContainer
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeout
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

class AutonomousControl(private val robotContainer: RobotContainer) {
    private val autonNT = NetworkTableInstance.getDefault().getTable("PID")
    private val autoChooser = SendableChooser<AutoStrategy>()
    private val errorPub = autonNT.getDoubleTopic("error").publish()
    private val autonShooterControl = ShooterControl(robotContainer)
    private val pathGPT = PathGPT(robotContainer.drivetrain::driveDirection)


    private val autoStrategies = listOf(
        BackUpAutoStrategy("THIS ONE"),
        MowingAutoStrategy(
            "Red Top Sweep Middle Return", Pose2d(
                Translation2d(8.679983, 7.038161),
                Rotation2d.fromDegrees(-90.0),
            ), Pose2d(
                Translation2d(14.0, 7.325687),
                Rotation2d.fromDegrees(180.0),
            ), Pose2d(
                Translation2d(15.8, 7.325687),
                Rotation2d.fromDegrees(180.0),
            )
        ),
        MowingAutoStrategy(
            "Red Bottom Sweep Middle Return",
            Pose2d(Translation2d(8.679983, 0.800000), Rotation2d.fromDegrees(-90.0)),
            Pose2d(Translation2d(14.000000, 0.716969), Rotation2d.fromDegrees(180.0)),
            Pose2d(Translation2d(15.800000, 0.716969), Rotation2d.fromDegrees(180.0)),
        ),
        MowingAutoStrategy(
            "Blue Top Sweep Middle Return",
            Pose2d(Translation2d(7.833065, 7.038161), Rotation2d.fromDegrees(-90.0)),
            Pose2d(Translation2d(2.513048, 7.325687), Rotation2d.fromDegrees(0.0)),
            Pose2d(Translation2d(0.713048, 7.325687), Rotation2d.fromDegrees(0.0)),
        ),
        MowingAutoStrategy(
            "Blue Bottom Sweep Middle Return",
            Pose2d(Translation2d(7.833065, 0.800000), Rotation2d.fromDegrees(90.0)),
            Pose2d(Translation2d(2.513048, 0.716969), Rotation2d.fromDegrees(0.0)),
            Pose2d(Translation2d(0.713048, 0.716969), Rotation2d.fromDegrees(0.0)),
        ),
    )

//// Builder options persist. Clear this before building any later path command.
//		robotContainer.drivetrain.pathBuilder.withPoseReset({ ignored -> })
    // TODO: Overrides withPoseReset function to not reset on new path > implement

    init {
        autoChooser.setDefaultOption("No Auto", NoOpAutoStrategy("No Auto"))
        for (strategy in autoStrategies) {
            autoChooser.addOption(strategy.name, strategy)
        }
        SmartDashboard.putData("Auto Chooser", autoChooser)
    }

    private suspend fun testPID() {
        while (true) {
            shuffleX(3.0)
            delay(1000.milliseconds)
            shuffleX(-3.0)
            delay(1000.milliseconds)
        }
    }

    private suspend fun shuffleX(delta: Double) {
        awaitNewPose()
        val startingX = Localization.getCurrentPose()?.pose?.translation?.x ?: return
        val setpoint = startingX + delta
        // TODO: Make this + other one constants
        val controller = PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD)
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
        val controller = PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD)
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
        coroutineScope {
            launch { autonShooterControl.updateLoop() }
            launch {
                try {
                    autoChooser.selected.execute()
                } finally {
                    robotContainer.drivetrain.stop()
                    robotContainer.intake.stop()
                    robotContainer.shooter.stop()
                }
            }
        }

    }

    private interface AutoStrategy {
        val name: String
        suspend fun execute()
    }

    private class NoOpAutoStrategy(override val name: String) : AutoStrategy {
        override suspend fun execute() {}
    }

    private inner class BackUpAutoStrategy(override val name: String, private val distance: Double = 1.0) :
        AutoStrategy {
        override suspend fun execute() {
            try {
                val pose = awaitNewPose() ?: return
                val alliance = DriverStation.getAlliance().getOrNull() ?: DriverStation.Alliance.Blue
                val target: Pose2d
                if (alliance == DriverStation.Alliance.Blue) {
                    target = Pose2d(Translation2d(pose.pose.x - distance, pose.pose.y), Rotation2d(0.0))
                } else {
                    target = Pose2d(Translation2d(pose.pose.x + distance, pose.pose.y), Rotation2d(PI))
                }
                withTimeout(5.seconds) { pathGPT.driveTo(target) }
                robotContainer.drivetrain.stop()
                autonShooterControl.mode = ShooterControl.TargetingMode.AUTOMATIC
                robotContainer.shooter.waitForShooterSpeed()
                robotContainer.intake.intake()
                delay(5.seconds)
            } finally {
                robotContainer.drivetrain.stop()
                robotContainer.intake.stop()
                robotContainer.shooter.stop()
            }
        }
    }

    private inner class MowingAutoStrategy(
        override val name: String,
        val firstTarget: Pose2d,
        val secondTarget: Pose2d,
        val thirdTarget: Pose2d,
    ) : AutoStrategy {
        override suspend fun execute() {
            val middleTarget =
                Pose2d(
                    Translation2d(firstTarget.x, HALF_FIELD_WIDTH_METERS),
                    firstTarget.rotation
                )
            try {
                coroutineScope {
                    launch {
                        robotContainer.intake.stop()
                        autonShooterControl.mode = ShooterControl.TargetingMode.IDLE
                        awaitZone(Zone.MIDDLE_BOTTOM, Zone.MIDDLE_TOP)
                        robotContainer.intake.intake()
                        autonShooterControl.mode = ShooterControl.TargetingMode.AUTOMATIC
                    }
                    launch {
                        pathGPT.driveTo(firstTarget)
                        pathGPT.driveTo(middleTarget, 1.0)
                        pathGPT.driveTo(secondTarget)
                        pathGPT.driveTo(thirdTarget,1.0)
                    }
                }
                delay(5.seconds)
            } finally {
                autonShooterControl.mode = ShooterControl.TargetingMode.IDLE
                robotContainer.drivetrain.stop()
                robotContainer.intake.stop()
                robotContainer.shooter.stop()
            }
        }
    }
}
