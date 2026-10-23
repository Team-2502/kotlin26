package com.team2502.robot2026.control.test

import com.team2502.lib.CoroutineRobot.Companion.awaitDriverStationData
import com.team2502.robot2026.Constants.Shooter.HOOD_MAX_POSITION
import com.team2502.robot2026.Constants.Shooter.SHOOTER_MAX_VELOCITY
import com.team2502.robot2026.Constants.Shooter.SHOOTER_MIN_VELOCITY
import com.team2502.robot2026.control.common.ShotSolution
import com.team2502.robot2026.subsystems.RobotContainer
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.PI
import kotlin.math.cos

class TestControl(private val robotContainer: RobotContainer) {
	suspend fun run() {
		try {
			val startTime = System.nanoTime()
			while (true) {
				awaitDriverStationData()
				val currentTime = System.nanoTime() - startTime
				val cosine = cos((currentTime / 1e9) / (PI * 3))

				if (cosine > 0) {
					robotContainer.intake.intake()
				} else {
					robotContainer.intake.outtake()
				}

				val range = SHOOTER_MAX_VELOCITY - SHOOTER_MIN_VELOCITY
				robotContainer.shooter.aimAndShoot(
					ShotSolution(
						range / 2.0 + cosine * range / 4.0,
						HOOD_MAX_POSITION / 2.0 + cosine * HOOD_MAX_POSITION / 4.0,
						Rotation2d(cosine * PI / 2.0),
						null,
					)
				)

				robotContainer.drivetrain.driveDirection(1.0, 1.0, PI / 6.0)
			}
		} finally {
			robotContainer.stop()
		}
	}
}
