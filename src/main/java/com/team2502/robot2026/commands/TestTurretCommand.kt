package com.team2502.robot2026.commands

import com.team2502.robot2026.RobotContainer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import java.util.function.DoubleSupplier
import kotlin.math.PI



fun setTurretCommand(double: DoubleSupplier): Command {
    val turretSubsystem = RobotContainer.INSTANCE.turretSubsystem
    return Commands.run({turretSubsystem.pointTo(Rotation2d(double.asDouble * PI/2))}, turretSubsystem)
}

fun setTurretFieldAngleCommand(angle: Rotation2d): Command {
    val turretSubsystem = RobotContainer.INSTANCE.turretSubsystem
    return Commands.run({turretSubsystem.pointTo(angle - RobotContainer.INSTANCE.getPose().rotation)}, turretSubsystem)
}