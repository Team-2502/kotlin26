package com.team2502.robot2026.commands

import com.team2502.robot2026.Constants.Localization.POSE_ANTICIPATION_TIMESTEP_SECS
import com.team2502.robot2026.Constants.Localization.YAW_ANTICIPATION_TIMESTEP_SECS
import com.team2502.robot2026.Constants.Turret.DISTANCE_SCALAR_SMUDGE_METERS
import com.team2502.robot2026.Constants.Turret.ORIGIN_TO_TURRET_CENTER_X
import com.team2502.robot2026.Constants.Turret.ORIGIN_TO_TURRET_CENTER_Y
import com.team2502.robot2026.Constants.Weights.COMMANDED_VELOCITY_WEIGHT
import com.team2502.robot2026.RobotContainer
import com.team2502.robot2026.subsystems.ShooterSubsystem
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import java.util.function.DoubleSupplier

fun shootCommand(double: DoubleSupplier): Command {
    val shooterSubsystem = RobotContainer.INSTANCE.shooterSubsystem

    return Commands.runEnd({
        shooterSubsystem.setShooterSpeed(double.asDouble * 70 + 30)
        shooterSubsystem.setHoodPosition(1.476562)
    }, {
        shooterSubsystem.stop()
    },  shooterSubsystem)
}

