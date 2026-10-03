// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2502.robot2026

import com.ctre.phoenix6.Utils
import com.team2502.robot2026.Constants.Localization.LIMELIGHT_FRONT_NAME
import com.team2502.robot2026.Constants.Localization.LIMELIGHT_SIDE_NAME
import com.team2502.robot2026.commands.shootCommand
import com.team2502.robot2026.subsystems.TargetingSubsystem
import com.team2502.robot2026.subsystems.VisionSubsystem
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain.*
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import java.util.concurrent.atomic.AtomicReference

class Robot : TimedRobot() {
    private var autonomousCommand: Command? = null
    private val robotContainer = RobotContainer()

    init {
        robotContainer.initialize()
    }

    override fun robotPeriodic() {
        robotContainer.update()
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        autonomousCommand = robotContainer.autonomousCommand
        robotContainer.reset()
        CommandScheduler.getInstance().schedule(autonomousCommand)
    }

    override fun autonomousPeriodic() {
    }

    override fun autonomousExit() {}

    override fun teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand!!.cancel()
        }
        robotContainer.reset()
    }

    override fun teleopPeriodic() {

    }

    override fun teleopExit() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()


    }

    override fun testPeriodic() {}

    override fun testExit() {}
}
