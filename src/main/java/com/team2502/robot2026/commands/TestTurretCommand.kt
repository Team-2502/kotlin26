package com.team2502.robot2026.commands

import com.team2502.robot2026.RobotContainer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

fun setTurretCommand(angle: Rotation2d): Command {
    val turretSubsystem = RobotContainer.INSTANCE.turretSubsystem
    return Commands.run({turretSubsystem.pointTo(angle)}, turretSubsystem)
}