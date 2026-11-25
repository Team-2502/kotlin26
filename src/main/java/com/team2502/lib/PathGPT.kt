// no gpt usage :)
package com.team2502.lib

import com.team2502.lib.Localization.awaitNewPose
import com.team2502.robot2026.Constants.Auto.ROTATION_KD
import com.team2502.robot2026.Constants.Auto.ROTATION_KI
import com.team2502.robot2026.Constants.Auto.ROTATION_KP
import com.team2502.robot2026.Constants.Auto.TRANSLATION_KD
import com.team2502.robot2026.Constants.Auto.TRANSLATION_KI
import com.team2502.robot2026.Constants.Auto.TRANSLATION_KP
import com.team2502.robot2026.control.autonomous.Nodes
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import kotlin.math.absoluteValue
import kotlin.math.sqrt

class PathGPT(private val driveFunction: (Double, Double, Double) -> Unit) {
    private val translationTuner = PIDTuner("translation", TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD)
    private val rotationTuner = PIDTuner("rotation", ROTATION_KP, ROTATION_KI, ROTATION_KD)
    private var translationController = PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD)
    private var rotationController = PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD)

    suspend fun driveTo(
        goal: Pose2d,
        maxSpeed: Double = 1.0,
        translationTolerance: Double = 0.05,
        rotationTolerance: Double = 0.05
    ) {
        println("checkpoint 1")
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

        println("checkpoint 2")
        for (index in points.indices) {
            if (index + 1 >= points.size)
                break
            println("checkpoint 3")
            followSegment(
                points[index],
                points[index + 1],
                points.getOrNull(index + 2),
                translationTolerance,
                rotationTolerance,
                maxSpeed,
            )
        }
    }

    private suspend fun followSegment(
        start: Pair<Translation2d, Rotation2d>,
        target: Pair<Translation2d, Rotation2d>,
        nextTarget: Pair<Translation2d, Rotation2d>?,
        translationTolerance: Double,
        rotationTolerance: Double,
        maxSpeed: Double
    ) {
        translationController = translationTuner.create()
        rotationController = rotationTuner.create()
        val loopCount = if (nextTarget != null) 20 else 1
        var goodCount = 0.0
        while (true) {
            println("checkpoint 4")
            val pose = awaitNewPose()?.pose ?: break
            println("checkpoint 5")
            if (withinTolerance(pose, target, translationTolerance, rotationTolerance)) {
                goodCount++
            } else {
                goodCount = 0.0
            }
            if (goodCount > loopCount) break
            println("checkpoint 6")

            val error = Pose2d(target.first, target.second) - pose
            translationTuner.reportError(sqrt(error.translation.dot(error.translation)))
            rotationTuner.reportError(error.rotation.degrees)
            val speeds = enforceSpeedLimit(calculatePID(error), maxSpeed)
            println("speeds: $speeds")
            driveFunction(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond)
        }
    }

    private fun withinTolerance(
        pose: Pose2d,
        target: Pair<Translation2d, Rotation2d>,
        translationTolerance: Double,
        rotationTolerance: Double,
    ): Boolean {
        val error = pose - Pose2d(target.first, target.second)
        return ((error.translation.getDistance(Translation2d()) < translationTolerance) &&
                (error.rotation.radians.absoluteValue < rotationTolerance))
    }

    private fun calculatePID(error: Transform2d): ChassisSpeeds {
        val gainX = translationController.calculate(error.x)
        val gainY = translationController.calculate(error.y)
        val gainRot = rotationController.calculate(error.rotation.radians)

        return ChassisSpeeds(-gainX, -gainY, gainRot)
    }

    private fun enforceSpeedLimit(
        speeds: ChassisSpeeds,
        maxSpeed: Double
    ): ChassisSpeeds {
        val gainX = speeds.vxMetersPerSecond
        val gainY = speeds.vyMetersPerSecond
        val gainRot = speeds.omegaRadiansPerSecond

        val pathSpeedSquared = gainX * gainX + gainY * gainY
        if (pathSpeedSquared > maxSpeed * maxSpeed) {
            val pathSpeed = sqrt(pathSpeedSquared)
            return ChassisSpeeds(
                gainX / pathSpeed * maxSpeed,
                gainY / pathSpeed * maxSpeed,
                gainRot
            )
        } else {
            return ChassisSpeeds(gainX, gainY, gainRot)
        }
    }
}