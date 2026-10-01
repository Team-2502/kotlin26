package com.team2502.robot2026.commands

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import com.team2502.robot2026.Constants.Turret.ORIGIN_TO_TURRET_CENTER_X_INCHES
import com.team2502.robot2026.Constants.Turret.ORIGIN_TO_TURRET_CENTER_Y_INCHES
import com.team2502.robot2026.Constants.Weights.COMMANDED_VELOCITY_WEIGHT
import com.team2502.robot2026.RobotContainer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

fun shootCommand(): Command {
    val turretSubsystem = RobotContainer.INSTANCE.turretSubsystem
    val shooterSubsystem = RobotContainer.INSTANCE.shooterSubsystem
    val driveSubsystem = RobotContainer.INSTANCE.drivetrainSubsystem
    val targetingSubsystem = RobotContainer.INSTANCE.targetingSubsystem

    return Commands.run({
        val currentPose = driveSubsystem.state.Pose
        val target = targetingSubsystem.currentTarget
        val currentFlywheelSpeed = shooterSubsystem.flywheelSpeed()

        //TODO: Future Pose
        val futurePose = currentPose

        val vectorToTurretCenter = Translation2d(
            ORIGIN_TO_TURRET_CENTER_X_INCHES,
            ORIGIN_TO_TURRET_CENTER_Y_INCHES,
        ).rotateBy(Rotation2d(futurePose.rotation.radians))
        val turretPose = futurePose.translation + vectorToTurretCenter

        val velocityVector = Translation2d(0.0, 0.0)
        val commandedVelocityVector = Translation2d(0.0, 0.0)

        val weightedVelocityX =
            velocityVector.x * (1.0 - COMMANDED_VELOCITY_WEIGHT) + (commandedVelocityVector.x * COMMANDED_VELOCITY_WEIGHT)
        val weightedVelocityY =
            velocityVector.y * (1.0 - COMMANDED_VELOCITY_WEIGHT) + (commandedVelocityVector.y * COMMANDED_VELOCITY_WEIGHT)
        val weightedVelocity = Translation2d(weightedVelocityX, weightedVelocityY)

        // target-relative = tr
        val trVelocity = targetRelativeVelocity(weightedVelocity, futurePose, target.position.translation)
        val currentDist = currentPose.translation.getDistance(target.position.translation)
        val futureDist = futurePose.translation.getDistance(target.position.translation)

        val flywheelSpeed: Double
        val hood: Double
        val angle: Rotation2d

        if (target.is_hub) {
            flywheelSpeed = predictHubSpeed(futureDist, trVelocity.x, trVelocity.y)
            hood = predictHubHood(futureDist, trVelocity.x, trVelocity.y, currentFlywheelSpeed)
            angle = getAngleTo(turretPose, target.position.translation) +
                    Rotation2d(predictYaw(currentDist, trVelocity.x, trVelocity.y, currentFlywheelSpeed, hood))
        } else {
            flywheelSpeed = predictPassSpeed(futureDist, trVelocity.x, trVelocity.y)
            hood = predictPassHood(futureDist, trVelocity.x, trVelocity.y, currentFlywheelSpeed)
            angle = getAngleTo(turretPose, target.position.translation) +
                    Rotation2d(predictYaw(currentDist, trVelocity.x, trVelocity.y, currentFlywheelSpeed, hood))
        }

        shooterSubsystem.setShooterSpeed(flywheelSpeed)
        shooterSubsystem.setHoodPosition(hood)
        turretSubsystem.pointTo(angle)
    }, turretSubsystem, shooterSubsystem)
}

fun targetRelativeVelocity(
    velocity: Translation2d,
    robotPose: Pose2d,
    target: Translation2d
): Translation2d {
    val toTarget = target.minus(robotPose.translation)

    val yUnitVector = toTarget.div(toTarget.norm)
    val xUnitVector = Translation2d(
        yUnitVector.y,
        -yUnitVector.x
    )

    val x = xUnitVector.x * velocity.x +
            xUnitVector.y * velocity.y
    val y = yUnitVector.x * velocity.x +
            yUnitVector.y * velocity.y

    return Translation2d(x, y)
}


fun predictHubSpeed(
    dist: Double,
    vx: Double,
    vy: Double
): Double {
    val vx2 = vx * vx
    val vy2 = vy * vy
    val vxvy2 = vx2 + vy2
    val dist2 = dist * dist

    val speed =
        48.26402033741077 +
                1.45313626590204 * dist +
                0.28497933990905 * dist2 +
                2.08552942755273 * vy -
                0.87492962508219 * vy * dist -
                0.0310463487289 * vy * dist2 +
                0.59753175991412 * vxvy2 +
                0.02438070724366 * vxvy2 * dist

    return speed.coerceIn(30.0, 100.0)
}

fun predictHubHood(
    dist: Double,
    vx: Double,
    vy: Double,
    speed: Double
): Double {
    val flywheel = speed * 0.14577997574526508
    val vx2 = vx * vx
    val vy2 = vy * vy
    val dist2 = dist * dist

    val term1 = dist + flywheel
    val term2 = flywheel + vy

    if (abs(term1) < 1e-9 || abs(term2) < 1e-9) {
        return 0.0
    }

    val invTerm1 = 1.0 / term1
    val invTerm2 = 1.0 / term2

    val ux = vx

    val uy =
        2.31930632574723 +
                0.87080314191958 * dist -
                0.55831432371477 * flywheel -
                1.00846814433622 * vy +
                0.01653743700391 * dist2 -
                0.04748169042816 * dist * flywheel +
                0.03367359917108 * flywheel * flywheel +
                0.00499141699947 * vy * flywheel +
                0.01341946850842 * vy * dist -
                0.00095734093026 * vy * dist * flywheel +
                0.02610786512318 * vx2 -
                0.00195469419643 * vx2 * flywheel -
                0.00126738212072 * vy2 +
                0.08166170069528 * hypot(vx, vy) * invTerm1 +
                2.76899075054095 * dist * invTerm2

    val uz =
        0.09936205098565 +
                0.02104813399762 * dist +
                1.11269483003193 * flywheel +
                0.19605481403508 * vy -
                0.02942888380699 * dist2 +
                0.02231266584791 * dist * flywheel -
                0.00832592641409 * flywheel * flywheel -
                0.01771030219332 * vy * flywheel +
                0.0854337955929 * vy * dist -
                0.00232108908698 * vy * dist * flywheel -
                0.09090984838824 * vx2 +
                0.00395729340336 * vx2 * flywheel -
                0.04211126605481 * vy2 -
                1.03180838142139 * hypot(vx, vy) * invTerm1 -
                3.13694287243635 * dist * invTerm2

    val angle =
        Math.toDegrees(atan2(uz, hypot(ux, uy))) /
                -12.56207643932571 +
                5.565478313566728

    return angle + 0.17
}

fun predictPassSpeed(
    dist: Double,
    vx: Double,
    vy: Double
): Double {
    val vx2 = vx * vx
    val vy2 = vy * vy
    val vxvy2 = vx2 + vy2
    val dist2 = dist * dist

    val speed =
        35.2969372164117 +
                1.65508715011467 * dist +
                0.3108259624189 * dist2 -
                1.46459651949985 * vy -
                0.37911499468948 * vy * dist -
                0.03985365245764 * vy * dist2 +
                0.14096777928633 * vxvy2 +
                0.03920102226153 * vxvy2 * dist

    return speed.coerceIn(30.0, 100.0)
}

fun predictPassHood(
    dist: Double,
    vx: Double,
    vy: Double,
    speed: Double
): Double {
    val flywheel = speed * 0.14577997574526508
    val vx2 = vx * vx
    val vy2 = vy * vy
    val dist2 = dist * dist

    val term1 = dist + flywheel
    val term2 = flywheel + vy

    if (abs(term1) < 1e-9 || abs(term2) < 1e-9) {
        return 0.0
    }

    val invTerm1 = 1.0 / term1
    val invTerm2 = 1.0 / term2

    val ux = vx

    val uy =
        2.03675058003524 +
                0.95267923340142 * dist -
                0.67709074682133 * flywheel -
                0.91681616131232 * vy +
                0.02489338139897 * dist2 -
                0.06862696241705 * dist * flywheel +
                0.04705358150443 * flywheel * flywheel -
                0.00308297663015 * vy * flywheel -
                0.00664554065875 * vy * dist +
                0.00046349869258 * vy * dist * flywheel +
                0.07717481930137 * vx2 -
                0.00563683887608 * vx2 * flywheel +
                0.00133046985465 * vy2 -
                0.29936462144772 * hypot(vx, vy) * invTerm1 +
                3.00745663885683 * dist * invTerm2

    val uz =
        0.36071337471016 -
                0.29550545777051 * dist +
                1.1566936615387 * flywheel +
                0.13254521320313 * vy -
                0.03946110656003 * dist2 +
                0.05787971099762 * dist * flywheel -
                0.0158577396631 * flywheel * flywheel -
                0.01459058154521 * vy * flywheel +
                0.12068032746825 * vy * dist -
                0.0044095155692 * vy * dist * flywheel -
                0.13875113842158 * vx2 +
                0.00728004615683 * vx2 * flywheel -
                0.04725195984191 * vy2 -
                0.53006481464723 * hypot(vx, vy) * invTerm1 -
                2.7343768311302 * dist * invTerm2

    val angle =
        Math.toDegrees(atan2(uz, hypot(ux, uy))) /
                -12.56207643932571 +
                5.565478313566728

    return angle + 0.17
}

fun predictYaw(
    dist: Double,
    vx: Double,
    vy: Double,
    speed: Double,
    hood: Double
): Double {
    val hoodCalibrated = hood - 0.17

    val launchVelocity =
        speed * (
                0.14577997574526508 -
                        0.00015049121874030828 * hoodCalibrated
                )

    val launchAngle =
        Math.toRadians(
            69.91396399643477 -
                    12.56207643932571 * hoodCalibrated
        )

    val tof =
        0.1019367991845056 *
                (launchVelocity * kotlin.math.sin(launchAngle)) +
                0.8675195210938735

    if (abs(tof) < 1e-9) {
        return 0.0
    }

    return atan2(
        vx,
        1.34212043514101 * (dist / tof) -
                vy +
                0.24667467825382
    )
}

fun getAngleTo(pose: Translation2d, target: Translation2d): Rotation2d =
    target.minus(pose).angle