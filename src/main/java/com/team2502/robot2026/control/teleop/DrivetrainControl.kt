package com.team2502.robot2026.control.teleop

import com.team2502.lib.Localization.awaitNewPose
import com.team2502.robot2026.Constants
import com.team2502.robot2026.subsystems.RobotContainer
import kotlin.math.PI
import kotlin.math.roundToLong

class DrivetrainControl(private val robotContainer: RobotContainer) {

	suspend fun updateLoop() {
		try {
			while (true) {
				val pose = awaitNewPose()
				val rotationRate: Double
				if (robotContainer.driverRightJoystick.leftThumbButton && pose != null) {
					val targetAngle = (pose.pose.rotation.radians / (PI / 2.0)).roundToLong() * PI / 2.0
					rotationRate = robotContainer.drivetrain.choreoThetaController.calculate(
						pose.pose.rotation.radians,
						targetAngle
					)
				} else {
					rotationRate =
						robotContainer.driverRightJoystick.twist * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
				}

				val velX =
					robotContainer.driverLeftJoystick.y * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND
				val velY =
					-robotContainer.driverLeftJoystick.x * Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND
				// NEEDS TO RENABLE TELEOP FOR ALLIANCE TO FLIP
				robotContainer.drivetrain.driveDirection(velX, velY, rotationRate)
			}
		} finally {
			robotContainer.drivetrain.stop()
		}
	}
}