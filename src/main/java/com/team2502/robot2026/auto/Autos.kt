package com.team2502.robot2026.auto

import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import com.team2502.robot2026.RobotContainer
import com.team2502.robot2026.commands.runIntakeCommand
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain
import edu.wpi.first.wpilibj2.command.Commands

class Autos(
    private val drive: CommandSwerveDrivetrain
) {

    private val autoFactory = AutoFactory(
        RobotContainer.INSTANCE::getPose,
        drive::resetChoreoPose,
        drive::followTrajectory,
        true,
        drive
    )

    fun path(): AutoRoutine {
        val routine = autoFactory.newRoutine("path")

        val path = routine.trajectory("path")

        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd()
            )
        )

        path.active().whileTrue(
            runIntakeCommand()
        )

        //path.done().onTrue()

        return routine
    }
}
