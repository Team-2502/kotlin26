package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityDutyCycle
import com.team2502.lib.config.create
import com.team2502.robot2026.Constants
import com.team2502.robot2026.RobotCAN.HOOD
import com.team2502.robot2026.RobotCAN.RAMP
import com.team2502.robot2026.RobotCAN.SHOOTER_LEFT
import com.team2502.robot2026.RobotCAN.SHOOTER_RIGHT
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.max
import kotlin.math.min

class ShooterSubsystem : SubsystemBase() {
    private val shooterLeftMotor = SHOOTER_LEFT.create()
    private val shooterRightMotor = SHOOTER_RIGHT.create()
    private val shooterHoodMotor = HOOD.create()

    private val shooterVelocityDutyCycleRequest = VelocityDutyCycle(0.0)
    private val shooterPositionRequest = PositionVoltage(0.0)

    fun setShooterSpeed(velocity: Double) {

        shooterLeftMotor.setControl(shooterVelocityDutyCycleRequest.withVelocity(velocity))
        shooterRightMotor.setControl(shooterVelocityDutyCycleRequest.withVelocity(velocity))
    }

    fun setHoodPosition(position: Double) {
        val clampedPosition =
            max(Constants.Shooter.HOOD_MIN_POSITION, min(Constants.Shooter.HOOD_MAX_POSITION, position))
        shooterHoodMotor.setControl(shooterPositionRequest.withPosition(clampedPosition))
    }

    fun flywheelSpeed() : Double {
        return shooterLeftMotor.velocity.valueAsDouble
    }

    fun stopShooter() {
        shooterLeftMotor.stopMotor()
        shooterRightMotor.stopMotor()
        shooterHoodMotor.stopMotor()
    }

    fun stopHood() {
        shooterHoodMotor.stopMotor()
    }
}