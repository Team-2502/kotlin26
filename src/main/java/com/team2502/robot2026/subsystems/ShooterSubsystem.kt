package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityDutyCycle
import com.team2502.lib.FileZero
import com.team2502.lib.config.create
import com.team2502.robot2026.Constants
import com.team2502.robot2026.Constants.Turret.ABS_TO_RELATIVE_RATIO
import com.team2502.robot2026.Constants.Turret.TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS
import com.team2502.robot2026.RobotCAN.HOOD
import com.team2502.robot2026.RobotCAN.SHOOTER_LEFT
import com.team2502.robot2026.RobotCAN.SHOOTER_RIGHT
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.io.File
import kotlin.math.max
import kotlin.math.min

class ShooterSubsystem : SubsystemBase() {
    private val shooterLeftMotor = SHOOTER_LEFT.create()
    private val shooterRightMotor = SHOOTER_RIGHT.create()
    private val shooterHoodMotor = HOOD.create()

    var hoodZeroPosition: Rotation2d? = null
    private val shooterVelocityDutyCycleRequest = VelocityDutyCycle(0.0)
    private val shooterPositionRequest = PositionVoltage(0.0)

    fun initialize() {
        hoodZeroPosition = Rotation2d.fromRotations(FileZero(
            File("/tmp/hood_zero"),
            {
                shooterHoodMotor.position.valueAsDouble
            }
        ))
    }

    fun setShooterSpeed(velocity: Double) {
        shooterLeftMotor.setControl(shooterVelocityDutyCycleRequest.withVelocity(velocity))
        shooterRightMotor.setControl(shooterVelocityDutyCycleRequest.withVelocity(velocity))
    }

    fun setHoodPosition(position: Double) {
        val hoodZero = this.hoodZeroPosition
        if (hoodZero == null) {
            print("[WARNING]: hoodZeroPosition was null when pointTo was called")
            return
        }

        val clampedPosition =
            max(Constants.Shooter.HOOD_MIN_POSITION, min(Constants.Shooter.HOOD_MAX_POSITION, position))
        shooterHoodMotor.setControl(shooterPositionRequest.withPosition(clampedPosition + hoodZero.rotations))
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