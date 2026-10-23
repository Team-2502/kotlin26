// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2502.robot2026

import com.team2502.lib.CoroutineRobot
import com.team2502.robot2026.control.autonomous.AutonomousControl
import com.team2502.robot2026.control.teleop.TeleopControl
import com.team2502.robot2026.control.test.TestControl
import com.team2502.robot2026.subsystems.RobotContainer

class Robot : CoroutineRobot() {
	private val robotContainer = RobotContainer()
	private val autonomousControl = AutonomousControl(robotContainer)
	private val teleopControl = TeleopControl(robotContainer)
	private val testControl = TestControl(robotContainer)

	override fun robotInit() {
		robotContainer.initialize()
	}

	override suspend fun runBackgroundRobot() {
		robotContainer.updateLoop()
	}

	override suspend fun runAutonomous() {
		autonomousControl.run()
	}

	override suspend fun runTeleop() {
		teleopControl.run()
	}

	override suspend fun runTest() {
		testControl.run()
	}

	override suspend fun runDisabled() {
		robotContainer.stop()
	}

}
