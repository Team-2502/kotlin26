package com.team2502.robot2026.control.teleop

import com.team2502.robot2026.Constants.Field.BLUE_HUB_X_METERS
import com.team2502.robot2026.Constants.Field.HALF_FIELD_WIDTH_METERS
import com.team2502.robot2026.Constants.Field.RED_HUB_X_METERS
import com.team2502.lib.CoroutineRobot.Companion.awaitDriverStationData
import com.team2502.lib.Localization.awaitNewPose
import com.team2502.robot2026.control.common.FieldTarget
import com.team2502.robot2026.control.common.Zone
import com.team2502.robot2026.control.common.calculateShot
import com.team2502.robot2026.subsystems.RobotContainer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlin.jvm.optionals.getOrNull

class ShooterControl(private val robotContainer: RobotContainer) {
	private var mode = TargetingMode.IDLE

	private val targetingNT = NetworkTableInstance.getDefault().getTable("targetingTable")
	private val modePub = targetingNT.getStringTopic("modePub").publish()
	private val targetNamePub = targetingNT.getStringTopic("targetNamePub").publish()
	private val currentZonePub = targetingNT.getStringTopic("currentZonePub").publish()

	suspend fun updateLoop() {
		coroutineScope {
			launch { updateOperatorLoop() }
			launch { updateControlLoop() }
		}
	}

	private suspend fun updateOperatorLoop() {
		try {
			while (true) {
				awaitDriverStationData()

				if (robotContainer.operatorJoystick.leftBaseBottomMiddle) {
					mode = TargetingMode.IDLE
					modePub.set("IDLE")
				} else if (robotContainer.operatorJoystick.leftBaseBottomRight) {
					mode = TargetingMode.AUTOMATIC
					modePub.set("AUTOMATIC")
				}
			}
		} finally {
			mode = TargetingMode.IDLE
			modePub.set("IDLE")
		}
	}

	private suspend fun updateControlLoop() {
		try {
			while (true) {
				val pose = awaitNewPose()
				if (pose == null) {
					robotContainer.drivetrain.speedLimit = null
					robotContainer.shooter.stop()
					continue
				}

				val target = updateTarget(pose.pose)
				if (mode == TargetingMode.AUTOMATIC && target != null) {
					val currentFlywheelSpeed = robotContainer.shooter.flywheelSpeed()
					val shot = calculateShot(pose, currentFlywheelSpeed, target)
					robotContainer.shooter.aimAndShoot(shot)
					robotContainer.drivetrain.speedLimit = shot.speedLimit
				} else {
					robotContainer.drivetrain.speedLimit = null
					robotContainer.shooter.stop()
				}
			}
		} finally {
			mode = TargetingMode.IDLE
			robotContainer.drivetrain.speedLimit = null
			robotContainer.shooter.stop()
		}
	}

	private fun updateTarget(pose: Pose2d): FieldTarget? { //TODO: telemetry targeting
		val alliance = DriverStation.getAlliance().getOrNull()
		if (alliance == null) {
			targetNamePub.set("No Alliance")
			currentZonePub.set("No Alliance")
			return null
		}

		val currentZone = if (pose.y < HALF_FIELD_WIDTH_METERS) {
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

		val target = if (alliance == DriverStation.Alliance.Blue) {
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

		targetNamePub.set(target.name)
		currentZonePub.set(currentZone.name)
		return target
	}

	enum class TargetingMode {
		AUTOMATIC, TELEMETRY, IDLE
	}


}