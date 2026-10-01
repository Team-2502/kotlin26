package com.team2502.robot2026.commands

import com.team2502.robot2026.Constants.Intake.UNJAM_CYCLE_TIME_SECONDS
import com.team2502.robot2026.RobotContainer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.repeatingSequence
import edu.wpi.first.wpilibj2.command.Commands.runEnd
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.WaitCommand

fun runIntakeCommand() : Command {
    val intakeSubsystem = RobotContainer.INSTANCE.intakeSubsystem

    println("hello my name is runIntakeCommand")

    return runEnd(
        intakeSubsystem::intake,
        intakeSubsystem::stop,
        intakeSubsystem,
    )
}

fun runOuttakeCommand() : Command {
    val intakeSubsystem = RobotContainer.INSTANCE.intakeSubsystem
    return runEnd(
        intakeSubsystem::outtake,
        intakeSubsystem::stop,
        intakeSubsystem,
    )
}

fun runUnjamCommand() : Command {
    val intakeSubsystem = RobotContainer.INSTANCE.intakeSubsystem

    return repeatingSequence(
        runOnce(intakeSubsystem::intake, intakeSubsystem),
        WaitCommand(UNJAM_CYCLE_TIME_SECONDS),
        runOnce(intakeSubsystem::outtake, intakeSubsystem),
        WaitCommand(UNJAM_CYCLE_TIME_SECONDS)
    ).finallyDo {
        _ -> intakeSubsystem.stop()
    }
}