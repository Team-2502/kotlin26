package com.team2502.robot2026.control.teleop

import com.team2502.robot2026.Constants.Intake.UNJAM_CYCLE_TIME_SECONDS
import com.team2502.lib.CoroutineRobot.Companion.awaitDriverStationData
import com.team2502.lib.CoroutineRobot.Companion.runWhile
import com.team2502.robot2026.subsystems.RobotContainer
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.delay
import kotlin.time.Duration.Companion.seconds

class IntakeControl(private val robotContainer: RobotContainer) {
	private val intakeNT = NetworkTableInstance.getDefault().getTable("intakeControlTable")
	private val operationPub = intakeNT.getStringTopic("intakeOperation").publish()

	suspend fun updateLoop() {
		try {
			while (true) {
				awaitDriverStationData()
				if (robotContainer.operatorJoystick.trigger || robotContainer.driverRightJoystick.trigger) {
					operationPub.set("INTAKE")
					robotContainer.intake.intake()
				} else if (robotContainer.operatorJoystick.middleThumbButton) {
					operationPub.set("OUTTAKE")
					robotContainer.intake.outtake()
				} else if (robotContainer.operatorJoystick.rightThumbButton) {
					operationPub.set("UNJAM")
					unjam()
				} else {
					operationPub.set("DISABLED")
					robotContainer.intake.stop()
				}
			}
		} finally {
			operationPub.set("DISABLED")
			robotContainer.intake.stop()
		}
	}

	private suspend fun unjam() {
		try {
			runWhile({ robotContainer.operatorJoystick.rightThumbButton }) {
				while (robotContainer.operatorJoystick.rightThumbButton) {
					robotContainer.intake.intake()
					delay(UNJAM_CYCLE_TIME_SECONDS.seconds)
					robotContainer.intake.outtake()
					delay(UNJAM_CYCLE_TIME_SECONDS.seconds)
				}
			}
		} finally {
			robotContainer.intake.stop()
		}
	}
}