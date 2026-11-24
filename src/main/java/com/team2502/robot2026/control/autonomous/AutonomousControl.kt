package com.team2502.robot2026.control.autonomous

import com.team2502.lib.Localization
import com.team2502.lib.Localization.awaitNewPose
import com.team2502.robot2026.Constants.Auto.BLINE_ROTATION_KD
import com.team2502.robot2026.Constants.Auto.BLINE_ROTATION_KI
import com.team2502.robot2026.Constants.Auto.BLINE_ROTATION_KP
import com.team2502.robot2026.Constants.Auto.BLINE_TRANSLATION_KD
import com.team2502.robot2026.Constants.Auto.BLINE_TRANSLATION_KI
import com.team2502.robot2026.Constants.Auto.BLINE_TRANSLATION_KP
import com.team2502.robot2026.Constants.Drivetrain.MAX_SPEED_METERS_PER_SECOND
import com.team2502.robot2026.Constants.Field.HALF_FIELD_LENGTH_METERS
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
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path
import frc.robot.lib.BLine.Path.DefaultGlobalConstraints
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeout
import java.util.Optional
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

class AutonomousControl(private val robotContainer: RobotContainer) {
    val pathBuilder = FollowPath.Builder(
        robotContainer.drivetrain.subsystem,
        { Localization.getCurrentPose()?.pose },
        { Localization.getCurrentPose()?.state?.Speeds },
        robotContainer.drivetrain::driveRobotRelative,
        PIDController(BLINE_TRANSLATION_KP, BLINE_TRANSLATION_KI, BLINE_TRANSLATION_KD),
        PIDController(BLINE_ROTATION_KP, BLINE_ROTATION_KI, BLINE_ROTATION_KD),
        PIDController(0.0, 0.0, 0.0),
    ).withTRatioBasedTranslationHandoffs(true)

    private val autonNT = NetworkTableInstance.getDefault().getTable("PID")
    private val autoChooser = SendableChooser<AutoStrategy>()
    private val errorPub = autonNT.getDoubleTopic("error").publish()
    private val autonShooterControl = ShooterControl(robotContainer)

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

        Path.setDefaultGlobalConstraints(
            DefaultGlobalConstraints(
                MAX_SPEED_METERS_PER_SECOND,  // maxVelocityMetersPerSec
                7.0,  // maxAccelerationMetersPerSec2
                720.0,  // maxVelocityDegPerSec
                1500.0,  // maxAccelerationDegPerSec2
                0.05,  // endTranslationToleranceMeters
                2.0,  // endRotationToleranceDeg
                0.3 // intermediateHandoffRadiusMeters
            )
        )
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
        val controller = PIDController(BLINE_TRANSLATION_KP, BLINE_TRANSLATION_KI, BLINE_TRANSLATION_KD)
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
        val controller = PIDController(BLINE_ROTATION_KP, BLINE_ROTATION_KI, BLINE_ROTATION_KD)
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

    private suspend fun followPath(cmd: FollowPath) {
        var interrupted = true
        try {
            cmd.initialize()
            try {
                while (true) {
                    awaitNewPose()
                    cmd.execute()
                    if (cmd.isFinished)
                        break
                }
                interrupted = false
            } finally {
                cmd.end(interrupted)
            }
        } finally {
            robotContainer.drivetrain.stop()
        }
    }

    private suspend fun driveTo(goal: Pose2d, builder: FollowPath.Builder, maxSpeed: Double = MAX_SPEED_METERS_PER_SECOND) {
        // Calculate the route when the command starts,
        // not when RobotContainer is initialized.
        val start = Localization.getCurrentPose()?.pose
        if (start == null) {
            DriverStation.reportWarning("No Localization start position", false)
            return
        }
        val points = Nodes.findPath(start.translation, start.rotation, goal.translation, goal.rotation)

        if (points.size < 2) {
            DriverStation.reportWarning("No valid path to $goal", false)
            return
        }
        val elements = mutableListOf<Path.PathElement>()

        elements.add(Path.Waypoint(start.translation, start.rotation))
        for (i in 1 until points.lastIndex) {
            elements.add(Path.RotationTarget(points[i].second, 0.50))
            elements.add(Path.TranslationTarget(points[i].first, Optional.of(0.10)))
        }
        elements.add(Path.Waypoint(goal.translation, goal.rotation))

        val constraints = createVelocityConstraints(points, maxSpeed)
        val path = Path(elements, constraints)

        followPath(builder.build(path))
    }

    private fun createVelocityConstraints(
        points: List<Pair<Translation2d, Rotation2d>>, maxSpeed: Double
    ): Path.PathConstraints {
        val acceleration = 9.0

        // null means no local constraint (use global default).
        // Index 0 is the starting position and stays null.
        val speeds = MutableList<Double>(points.size) { maxSpeed }

        for (i in 1 until points.lastIndex) {
            val previous = points[i - 1].first
            val current = points[i].first
            val next = points[i + 1].first

            val incoming = current.minus(previous)
            val outgoing = next.minus(current)

            if (incoming.norm < 1e-6 || outgoing.norm < 1e-6)
                continue

            val angle = kotlin.math.abs(
                incoming.angle.minus(outgoing.angle).degrees
            )

            val cornerSpeed = when {
                angle >= 90.0 -> 1.0
                angle >= 45.0 -> 1.8
                angle >= 25.0 -> 2.5
                else -> maxSpeed
            }

            if (cornerSpeed >= maxSpeed) continue

            val brakingDistance =
                (maxSpeed * maxSpeed - cornerSpeed * cornerSpeed) /
                        (2.0 * acceleration)

            var distance = 0.0
            var index = i

            // Slow approaching targets, but never constrain
            // the starting waypoint (index 0).
            while (index >= 1 && distance < brakingDistance) {
                speeds[index] = minOf(
                    speeds[index] ?: maxSpeed,
                    cornerSpeed
                )

                distance += points[index - 1].first.getDistance(
                    points[index].first
                )
                index--
            }
        }

        // Only emit constraints for waypoints that need slowing.
        // Ordinal 0 is deliberately excluded.
        val ranges = (1 until points.size).mapNotNull { index ->
            speeds[index]?.let { speed ->
                Path.RangedConstraint(speed, index, index)
            }
        }

        return Path.PathConstraints().apply {
            if (ranges.isNotEmpty()) {
                setMaxVelocityMetersPerSec(*ranges.toTypedArray())
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
                withTimeout(5.seconds) { driveTo(target, pathBuilder) }
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
                        driveTo(firstTarget, pathBuilder)
                        driveTo(middleTarget, pathBuilder, 0.0)
                        driveTo(secondTarget, pathBuilder)
                        driveTo(thirdTarget, pathBuilder, 1.0)
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
