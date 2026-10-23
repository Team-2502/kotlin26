package com.team2502.robot2026.control.autonomous

import choreo.auto.AutoFactory
import com.team2502.lib.CoroutineRobot.Companion.followChoreoPath
import com.team2502.lib.CoroutineRobot.Companion.resetOdometry
import com.team2502.lib.Localization
import com.team2502.lib.Localization.awaitNewPose
import com.team2502.robot2026.control.common.FieldTarget
import com.team2502.robot2026.control.common.calculateShot
import com.team2502.robot2026.subsystems.RobotContainer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeout
import kotlin.jvm.optionals.getOrNull
import kotlin.time.Duration.Companion.seconds

class AutonomousControl(private val robotContainer: RobotContainer) {

	private val autoChooser = SendableChooser<String>()
	private val autoFactory = AutoFactory(
		{ Localization.getCurrentPose()?.pose },
		robotContainer.drivetrain::resetChoreoPose,
		robotContainer.drivetrain::followTrajectory,
		true,
		robotContainer.drivetrain.subsystem
	)

	init {
		autoChooser.setDefaultOption("red left", "red_left")
		autoChooser.addOption("red right", "red_right")
		autoChooser.addOption("red mid", "red_mid")

		autoChooser.addOption("blue left", "blue_left")
		autoChooser.addOption("blue right", "blue_right")
		autoChooser.addOption("blue mid", "blue_mid")

		SmartDashboard.putData("Auto Chooser", autoChooser)
	}

	suspend fun run() {
		val alliance = DriverStation.getAlliance().getOrNull()
		if (alliance == null) {
			System.err.println("ERROR: Not on any alliance!")
			return
		}
		try {
			coroutineScope {
				robotContainer.intake.stop()
				resetOdometry(autoFactory, autoChooser.selected)
				followChoreoPath(autoFactory, autoChooser.selected)
				robotContainer.drivetrain.stop()
				delay(2.seconds)
				val shooter = launch { shootAtHub(alliance) }
				withTimeout(5.seconds) { robotContainer.shooter.waitForShooterSpeed() }
				robotContainer.intake.intake()
				withTimeout(5.seconds) { shooter.join() }
			}
		} finally {
			robotContainer.drivetrain.stop()
			robotContainer.intake.stop()
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
