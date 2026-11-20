package com.team2502.robot2026.control.autonomous

import com.team2502.lib.CoroutineRobot.Companion.runCommand
import com.team2502.lib.Localization
import com.team2502.lib.Localization.awaitNewPose
import com.team2502.robot2026.Constants.Auto.BLINE_ROTATION_KD
import com.team2502.robot2026.Constants.Auto.BLINE_ROTATION_KI
import com.team2502.robot2026.Constants.Auto.BLINE_ROTATION_KP
import com.team2502.robot2026.Constants.Auto.BLINE_TRANSLATION_KD
import com.team2502.robot2026.Constants.Auto.BLINE_TRANSLATION_KI
import com.team2502.robot2026.Constants.Auto.BLINE_TRANSLATION_KP
import com.team2502.robot2026.control.common.FieldTarget
import com.team2502.robot2026.control.common.calculateShot
import com.team2502.robot2026.subsystems.RobotContainer
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path
import frc.robot.lib.BLine.Path.DefaultGlobalConstraints
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import java.util.Optional
import kotlin.jvm.optionals.getOrNull
import kotlin.math.absoluteValue
import kotlin.time.Duration.Companion.milliseconds

class AutonomousControl(private val robotContainer: RobotContainer) {
//    val pathBuilder = FollowPath.Builder(
//        robotContainer.drivetrain.subsystem,
//        { Localization.getCurrentPose()?.pose },
//        { Localization.getCurrentPose()?.state?.Speeds },
//        robotContainer.drivetrain::driveRobotRelative,
//        PIDController(BLINE_TRANSLATION_KP, BLINE_TRANSLATION_KI, BLINE_TRANSLATION_KD),
//        PIDController(BLINE_ROTATION_KP, BLINE_ROTATION_KI, BLINE_ROTATION_KD),
//        PIDController(0.0, 0.0, 0.0),
//    ).withTRatioBasedTranslationHandoffs(true)

    private val autonNT = NetworkTableInstance.getDefault().getTable("PID")
    private val translationkPSub = autonNT.getDoubleTopic("translationkP").getEntry(0.0)
    private val translationkISub = autonNT.getDoubleTopic("translationkI").getEntry(0.0)
    private val translationkDSub = autonNT.getDoubleTopic("translationkD").getEntry(0.0)
    private val yawkPSub = autonNT.getDoubleTopic("yawkP").getEntry(0.0)
    private val yawkISub = autonNT.getDoubleTopic("yawkI").getEntry(0.0)
    private val yawkDSub = autonNT.getDoubleTopic("yawkD").getEntry(0.0)
    private val ctkPSub = autonNT.getDoubleTopic("ctkP").getEntry(0.0)
    private val ctkISub = autonNT.getDoubleTopic("ctkI").getEntry(0.0)
    private val ctkDSub = autonNT.getDoubleTopic("ctkD").getEntry(0.0)
    private val errorPub = autonNT.getDoubleTopic("error").publish()


//// Builder options persist. Clear this before building any later path command.
//		robotContainer.drivetrain.pathBuilder.withPoseReset({ ignored -> })
    // TODO: Overrides withPoseReset function to not reset on new path > implement

    init {
        translationkPSub.setDefault(0.0)
        translationkISub.setDefault(0.0)
        translationkDSub.setDefault(0.0)
        yawkPSub.setDefault(0.0)
        yawkISub.setDefault(0.0)
        yawkDSub.setDefault(0.0)
        ctkPSub.setDefault(0.0)
        ctkISub.setDefault(0.0)
        ctkDSub.setDefault(0.0)

        errorPub.setDefault(0.0)

        Path.setDefaultGlobalConstraints(
            DefaultGlobalConstraints(
                4.5,  // maxVelocityMetersPerSec
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
        val alliance = DriverStation.getAlliance().getOrNull()
        if (alliance == null) {
            System.err.println("ERROR: Not on any alliance!")
            return
        }
        try {
            coroutineScope {
                while (true) {
                    val pathBuilder = FollowPath.Builder(
                        robotContainer.drivetrain.subsystem,
                        { Localization.getCurrentPose()?.pose },
                        { Localization.getCurrentPose()?.state?.Speeds },
                        robotContainer.drivetrain::driveRobotRelative,
                        PIDController(BLINE_TRANSLATION_KP, BLINE_TRANSLATION_KI, BLINE_TRANSLATION_KD),
                        PIDController(BLINE_ROTATION_KP, BLINE_ROTATION_KI, BLINE_ROTATION_KD),
                        PIDController(ctkPSub.get(), ctkISub.get(), ctkDSub.get()),
                    ).withTRatioBasedTranslationHandoffs(true)

                    driveTo(
                        Pose2d(
                            Translation2d(14.0, 2.5),
                            Rotation2d.fromDegrees(0.0),
                        ), pathBuilder
                    )
                    driveTo(
                        Pose2d(
                            Translation2d(10.0, 2.5),
                            Rotation2d.fromDegrees(180.0)
                        ), pathBuilder
                    )
                    delay(100.milliseconds)
                }
            }
        } finally {
            robotContainer.drivetrain.stop()
            robotContainer.intake.stop()
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

    private suspend fun driveTo(goal: Pose2d, builder: FollowPath.Builder) {
        println("drive to $goal")
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

        val constraints = createVelocityConstraints(points)
        val path = Path(elements, constraints)

        followPath(builder.build(path))
    }

    private fun createVelocityConstraints(
        points: List<Pair<Translation2d, Rotation2d>>
    ): Path.PathConstraints {
        val maxSpeed = 4.5
        val acceleration = 9.0

        // null means no local constraint (use global default).
        // Index 0 is the starting position and stays null.
        val speeds = MutableList<Double?>(points.size) { null }

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
