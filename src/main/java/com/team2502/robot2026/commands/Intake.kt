package com.team2502.robot2026.commands;

import com.team2502.robot2026.RobotContainer
import edu.wpi.first.wpilibj2.command.Command

class RunIntakeCommand : Command() {
    private val intakeSubsystem = RobotContainer.INSTANCE.intakeSubsystem
    init {
        addRequirements(intakeSubsystem)
    }

    override fun execute() {
        intakeSubsystem.intake()
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.stop()
    }
}
