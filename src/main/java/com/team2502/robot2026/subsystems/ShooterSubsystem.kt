package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.team2502.robot2026.Constants
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.max
import kotlin.math.min

class ShooterSubsystem : SubsystemBase() {
    private val shooterLeftMotor = TalonFX(Constants.Shooter.SHOOTER_LEFT_MOTOR_ID)
    private val shooterRightMotor = TalonFX(Constants.Shooter.SHOOTER_RIGHT_MOTOR_ID)
    private val shooterHoodMotor = TalonFX(Constants.Shooter.SHOOTER_HOOD_MOTOR_ID)

    private val shooterVelocityRequest = VelocityVoltage(0.0)
    private val shooterPositionRequest = PositionVoltage(0.0)

    fun setShooterSpeed(velocity: Double): Command {
        val clampedVelocity =
            max(Constants.Shooter.SHOOTER_MIN_VELOCITY, min(Constants.Shooter.SHOOTER_MAX_VELOCITY, velocity))
        return runOnce {
            shooterLeftMotor.setControl(shooterVelocityRequest.withVelocity(clampedVelocity))
            shooterRightMotor.setControl(shooterVelocityRequest.withVelocity(-clampedVelocity))
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