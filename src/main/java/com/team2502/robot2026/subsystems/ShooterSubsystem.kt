package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.team2502.robot2026.Constants
import com.team2502.robot2026.Constants.Shooter.SHOOTER_MAX_VELOCITY
import com.team2502.robot2026.Constants.Shooter.SHOOTER_MIN_VELOCITY
import com.team2502.robot2026.RobotCAN.HOOD
import com.team2502.robot2026.RobotCAN.SHOOTER_LEFT
import com.team2502.robot2026.RobotCAN.SHOOTER_RIGHT
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.max
import kotlin.math.min

class ShooterSubsystem : SubsystemBase() {
    private val shooterLeftMotor = TalonFX(SHOOTER_LEFT.canId)
    private val shooterRightMotor = TalonFX(SHOOTER_RIGHT.canId)
    private val shooterHoodMotor = TalonFX(HOOD.canId)

    private val shooterVelocityRequest = VelocityVoltage(0.0)
    private val shooterPositionRequest = PositionVoltage(0.0)

    init {
        shooterLeftMotor.configurator.apply(SHOOTER_LEFT.config.generate())
        shooterRightMotor.configurator.apply(SHOOTER_RIGHT.config.generate())
        shooterHoodMotor.configurator.apply(HOOD.config.generate())
    }

    fun setShooterSpeed(velocity: Double): Command {
        return runOnce {
            shooterLeftMotor.setControl(shooterVelocityRequest.withVelocity(velocity))
            shooterRightMotor.setControl(shooterVelocityRequest.withVelocity(-velocity))
        }
    }

    fun setHoodPosition(position: Double): Command {
        val clampedPosition =
            max(Constants.Shooter.HOOD_MIN_POSITION, min(Constants.Shooter.HOOD_MAX_POSITION, position))
        return runOnce { shooterHoodMotor.setControl(shooterPositionRequest.withPosition(clampedPosition)) }
    }

    fun stop(): Command {
        return runOnce {
            shooterLeftMotor.stopMotor()
            shooterRightMotor.stopMotor()
            shooterHoodMotor.stopMotor()
        }
    }
}