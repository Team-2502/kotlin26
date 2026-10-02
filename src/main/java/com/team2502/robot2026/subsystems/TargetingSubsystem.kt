package com.team2502.robot2026.subsystems

import com.team2502.robot2026.Constants.Field.BLUE_BOTTOM_PASS_OFFSET_X_METERS
import com.team2502.robot2026.Constants.Field.BLUE_BOTTOM_PASS_OFFSET_Y_METERS
import com.team2502.robot2026.Constants.Field.BLUE_HUB_X_METERS
import com.team2502.robot2026.Constants.Field.BLUE_HUB_Y_METERS
import com.team2502.robot2026.Constants.Field.BLUE_TOP_PASS_OFFSET_X_METERS
import com.team2502.robot2026.Constants.Field.BLUE_TOP_PASS_OFFSET_Y_METERS
import com.team2502.robot2026.Constants.Field.HALF_FIELD_WIDTH_METERS
import com.team2502.robot2026.Constants.Field.RED_BOTTOM_PASS_OFFSET_X_METERS
import com.team2502.robot2026.Constants.Field.RED_BOTTOM_PASS_OFFSET_Y_METERS
import com.team2502.robot2026.Constants.Field.RED_HUB_X_METERS
import com.team2502.robot2026.Constants.Field.RED_HUB_Y_METERS
import com.team2502.robot2026.Constants.Field.RED_TOP_PASS_OFFSET_X_METERS
import com.team2502.robot2026.Constants.Field.RED_TOP_PASS_OFFSET_Y_METERS
import com.team2502.robot2026.RobotContainer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import kotlin.jvm.optionals.getOrNull

enum class TargetingMode {
    AUTOMATIC, TELEMETRY, IDLE
}

class FieldTarget(var position: Pose2d, var is_hub: Boolean, var name: String) {
    companion object {
        val BLUE_HUB = FieldTarget(
            Pose2d(BLUE_HUB_X_METERS, BLUE_HUB_Y_METERS, Rotation2d(0.0)),
            true,
            "Blue Hub"
        )
        val RED_HUB = FieldTarget(
            Pose2d(RED_HUB_X_METERS, RED_HUB_Y_METERS, Rotation2d(0.0)),
            true,
            "Red Hub"
        )
        val BLUE_PASS_TOP = FieldTarget(
            Pose2d(
                BLUE_HUB_X_METERS + BLUE_TOP_PASS_OFFSET_X_METERS,
                BLUE_HUB_Y_METERS + BLUE_TOP_PASS_OFFSET_Y_METERS,
                Rotation2d(0.0)),
            false,
            "Blue Pass Top"
        )
        val BLUE_PASS_BOTTOM = FieldTarget(
            Pose2d(
                BLUE_HUB_X_METERS + BLUE_BOTTOM_PASS_OFFSET_X_METERS,
                BLUE_HUB_Y_METERS + BLUE_BOTTOM_PASS_OFFSET_Y_METERS,
                Rotation2d(0.0)),
            false,
            "Blue Pass Bottom"
        )
        val RED_PASS_TOP = FieldTarget(
            Pose2d(
                RED_HUB_X_METERS + RED_TOP_PASS_OFFSET_X_METERS,
                RED_HUB_Y_METERS + RED_TOP_PASS_OFFSET_Y_METERS,
                Rotation2d(0.0)),
            false,
            "Red Pass Top"
        )
        val RED_PASS_BOTTOM = FieldTarget(
            Pose2d(
                RED_HUB_X_METERS + RED_BOTTOM_PASS_OFFSET_X_METERS,
                RED_HUB_Y_METERS + RED_BOTTOM_PASS_OFFSET_Y_METERS,
                Rotation2d(0.0)),
            false,
            "Red Pass Bottom"
        )
    }
}

enum class Zone {
    BLUE_BOTTOM, BLUE_TOP, MIDDLE_BOTTOM, MIDDLE_TOP, RED_BOTTOM, RED_TOP
}

class TargetingSubsystem(private val robotContainer: RobotContainer) {
    var mode = TargetingMode.IDLE
    var currentTarget = FieldTarget(Pose2d(0.0, 0.0, Rotation2d(0.0)), false, "INITIALIZATION TARGET")
    var currentZone = Zone.BLUE_BOTTOM

    fun update() {
        val pose = robotContainer.drivetrainSubsystem.state.Pose;

        updateZone(pose)
        updateTarget()
    }

    private fun updateZone(pose: Pose2d) {
        currentZone = if (pose.y < HALF_FIELD_WIDTH_METERS) {
            when {
                pose.x < BLUE_HUB_X_METERS -> Zone.BLUE_BOTTOM
                pose.x < RED_HUB_X_METERS -> Zone.MIDDLE_BOTTOM
                else -> Zone.RED_BOTTOM
            }
        } else {
            when {
                pose.x < BLUE_HUB_X_METERS -> Zone.BLUE_TOP
                pose.x < RED_HUB_X_METERS -> Zone.MIDDLE_TOP
                else -> Zone.RED_TOP
            }
        }
    }

    private fun updateTarget() { //TODO: telemetry targeting
        val alliance = DriverStation.getAlliance().getOrNull() ?: return

        currentTarget = if (alliance == DriverStation.Alliance.Blue) {
            when (currentZone) {
                Zone.BLUE_TOP -> FieldTarget.BLUE_HUB
                Zone.BLUE_BOTTOM -> FieldTarget.BLUE_HUB
                Zone.MIDDLE_TOP -> FieldTarget.BLUE_PASS_TOP
                Zone.MIDDLE_BOTTOM -> FieldTarget.BLUE_PASS_BOTTOM
                Zone.RED_TOP -> FieldTarget.BLUE_PASS_TOP
                Zone.RED_BOTTOM -> FieldTarget.BLUE_PASS_BOTTOM
            }
        } else {
            when (currentZone) {
                Zone.BLUE_TOP -> FieldTarget.RED_PASS_TOP
                Zone.BLUE_BOTTOM -> FieldTarget.RED_PASS_BOTTOM
                Zone.MIDDLE_TOP -> FieldTarget.RED_PASS_TOP
                Zone.MIDDLE_BOTTOM -> FieldTarget.RED_PASS_BOTTOM
                Zone.RED_TOP -> FieldTarget.RED_HUB
                Zone.RED_BOTTOM -> FieldTarget.RED_HUB
            }
        }
    }
}