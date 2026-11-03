package com.team2502.robot2026.control.teleop

import com.team2502.robot2026.subsystems.RobotContainer
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch

class TeleopControl(robotContainer: RobotContainer) {
	private val drivetrainControl = DrivetrainControl(robotContainer)
	private val shooterControl = ShooterControl(robotContainer)
	private val intakeControl = IntakeControl(robotContainer)

	suspend fun run() {
		coroutineScope {
			launch { drivetrainControl.updateLoop() }
			launch { shooterControl.updateLoop() }
			launch { intakeControl.updateLoop() }
		}
	}
}
