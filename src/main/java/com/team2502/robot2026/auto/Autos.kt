package com.team2502.robot2026.auto

import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import com.team2502.robot2026.RobotContainer
import com.team2502.robot2026.commands.runIntakeCommand
import com.team2502.robot2026.commands.stopIntakeCommand
import com.team2502.robot2026.commands.shootCommand
import com.team2502.robot2026.subsystems.TargetingMode
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


        return routine
    }

    fun redLeft(): AutoRoutine {
        val routine = autoFactory.newRoutine("red_left")

        val path = routine.trajectory("red_left")
        
        routine.active().onTrue(
            Commands.sequence(
                stopIntakeCommand(),
                path.resetOdometry(),
                path.cmd()
            )
        )

        RobotContainer.INSTANCE.targetingSubsystem.mode = TargetingMode.AUTOMATIC

        path.doneDelayed(2.0).onTrue(shootCommand())
        path.doneDelayed(4.0).onTrue(runIntakeCommand())

        return routine
    }

    fun redRight(): AutoRoutine {
        val routine = autoFactory.newRoutine("red_right")

        val path = routine.trajectory("red_right")
        
        routine.active().onTrue(
            Commands.sequence(
                stopIntakeCommand(),
                path.resetOdometry(),
                path.cmd()
            )
        )

        RobotContainer.INSTANCE.targetingSubsystem.mode = TargetingMode.AUTOMATIC

        path.doneDelayed(2.0).onTrue(shootCommand())
        path.doneDelayed(4.0).onTrue(runIntakeCommand())

        return routine
    }

    fun blueLeft(): AutoRoutine {
        val routine = autoFactory.newRoutine("blue_left")

        val path = routine.trajectory("blue_left")
        
        routine.active().onTrue(
            Commands.sequence(
                stopIntakeCommand(),
                path.resetOdometry(),
                path.cmd()
            )
        )

        RobotContainer.INSTANCE.targetingSubsystem.mode = TargetingMode.AUTOMATIC

        path.doneDelayed(2.0).onTrue(shootCommand())
        path.doneDelayed(4.0).onTrue(runIntakeCommand())

        return routine
    }

    fun blueRight(): AutoRoutine {
        val routine = autoFactory.newRoutine("blue_right")

        val path = routine.trajectory("blue_right")
        
        routine.active().onTrue(
            Commands.sequence(
                stopIntakeCommand(),
                path.resetOdometry(),
                path.cmd()
            )
        )

        RobotContainer.INSTANCE.targetingSubsystem.mode = TargetingMode.AUTOMATIC

        path.doneDelayed(2.0).onTrue(shootCommand())
        path.doneDelayed(4.0).onTrue(runIntakeCommand())

        return routine
    }

    fun blueMid(): AutoRoutine {
        val routine = autoFactory.newRoutine("blue_mid")

        val path = routine.trajectory("blue_mid")
        
        routine.active().onTrue(
            Commands.sequence(
                stopIntakeCommand(),
                path.resetOdometry(),
                path.cmd()
            )
        )

        RobotContainer.INSTANCE.targetingSubsystem.mode = TargetingMode.AUTOMATIC

        path.doneDelayed(2.0).onTrue(shootCommand())
        path.doneDelayed(4.0).onTrue(runIntakeCommand())

        return routine
    }

    fun redMid(): AutoRoutine {
        val routine = autoFactory.newRoutine("red_mid")

        val path = routine.trajectory("red_mid")
        
        routine.active().onTrue(
            Commands.sequence(
                stopIntakeCommand(),
                path.resetOdometry(),
                path.cmd()
            )
        )

        RobotContainer.INSTANCE.targetingSubsystem.mode = TargetingMode.AUTOMATIC

        path.doneDelayed(2.0).onTrue(shootCommand())
        path.doneDelayed(4.0).onTrue(runIntakeCommand())

        return routine
    }
}
