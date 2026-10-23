package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityDutyCycle
import com.team2502.lib.CoroutineSubsystem
import com.team2502.lib.FileZero
import com.team2502.lib.config.create
import com.team2502.robot2026.Constants
import com.team2502.robot2026.Constants.Turret.ABS_TO_RELATIVE_RATIO
import com.team2502.robot2026.Constants.Turret.MOTOR_TO_TURRET_RATIO
import com.team2502.robot2026.Constants.Turret.TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS
import com.team2502.robot2026.Constants.Turret.TURRET_CLAMP
import com.team2502.robot2026.RobotCAN.HOOD
import com.team2502.robot2026.RobotCAN.SHOOTER_LEFT
import com.team2502.robot2026.RobotCAN.SHOOTER_RIGHT
import com.team2502.robot2026.RobotCAN.TURRET
import com.team2502.robot2026.RobotCAN.TURRET_ENCODER
import com.team2502.robot2026.control.common.ShotSolution
import edu.wpi.first.math.MathUtil.angleModulus
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.delay
import java.io.File
import kotlin.math.max
import kotlin.math.min
import kotlin.time.Duration.Companion.milliseconds

class ShooterSubsystem : CoroutineSubsystem {
    private val shooterLeftMotor = SHOOTER_LEFT.create()
    private val shooterRightMotor = SHOOTER_RIGHT.create()
    private val shooterHoodMotor = HOOD.create()
    private val turretMotor = TURRET.create()
    private val turretEncoder = TURRET_ENCODER.create()

    private var state = "INITIALIZED"
    private val shooterVelocityDutyCycleRequest = VelocityDutyCycle(0.0)
    private val shooterPositionRequest = PositionVoltage(0.0)
    private val turretPositionRequest = PositionVoltage(0.0)

    private val shooterNT = NetworkTableInstance.getDefault().getTable("shooterSubsystemTable")
    private val statePub = shooterNT.getStringTopic("state").publish()
    private val desiredShooterPub = shooterNT.getDoubleTopic("desiredShooterSpeed").publish()
    private val desiredHoodPub = shooterNT.getDoubleTopic("desiredHoodPosition").publish()
    private val desiredTurretPub = shooterNT.getDoubleTopic("desiredTurretAngle").publish()

    var hoodZeroPosition: Rotation2d? = null
    var turretZeroPosition: Rotation2d? = null

    override fun initialize() {
        hoodZeroPosition = Rotation2d.fromRotations(FileZero(File("/tmp/hood_zero")) {
            shooterHoodMotor.position.valueAsDouble
        })
        turretZeroPosition = Rotation2d.fromRotations(FileZero(File("/tmp/turret_zero")) {
            turretMotor.position.valueAsDouble +
                    (turretEncoder.absolutePosition.valueAsDouble - TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS) *
                    ABS_TO_RELATIVE_RATIO
        })
    }

    override fun periodic() {
        statePub.set(state)
        desiredShooterPub.set(shooterVelocityDutyCycleRequest.Velocity)
        desiredHoodPub.set(shooterPositionRequest.Position)
        desiredTurretPub.set(turretPositionRequest.Position)
    }

    override fun stop() {
        state = "STOPPED"
        shooterLeftMotor.stopMotor()
        shooterRightMotor.stopMotor()
        shooterHoodMotor.stopMotor()
        turretMotor.stopMotor()
    }

    fun aimAndShoot(shot: ShotSolution) {
        state = "RUNNING"
        setShooterSpeed(shot.flywheelSpeed)
        setHoodPosition(shot.hoodPosition)
        pointTo(shot.turretAngle)
    }

    private fun setShooterSpeed(velocity: Double) {
        shooterLeftMotor.setControl(shooterVelocityDutyCycleRequest.withVelocity(velocity))
        shooterRightMotor.setControl(shooterVelocityDutyCycleRequest.withVelocity(velocity))
    }

    private fun setHoodPosition(position: Double) {
        val hoodZero = this.hoodZeroPosition
        if (hoodZero == null) {
            print("[WARNING]: hoodZeroPosition was null when pointTo was called")
            return
        }

        val clampedPosition =
            max(Constants.Shooter.HOOD_MIN_POSITION, min(Constants.Shooter.HOOD_MAX_POSITION, position))
        shooterHoodMotor.setControl(shooterPositionRequest.withPosition(clampedPosition + hoodZero.rotations))
    }

    /** points turret to a ROBOT relative angle, with 0 being away from intake **/
    private fun pointTo(angle: Rotation2d) {
        val turretZero = this.turretZeroPosition
        if (turretZero == null) {
            print("[WARNING]: turretZeroPosition was null when pointTo was called")
            return
        }

        val currentPositon = turretMotor.position.valueAsDouble

        val wrappedAngle = Rotation2d(angleModulus(angle.radians))
        val targetPosition =
            ((wrappedAngle.rotations * MOTOR_TO_TURRET_RATIO) + turretZero.rotations)
                .coerceIn(currentPositon - TURRET_CLAMP, currentPositon + TURRET_CLAMP)
        turretMotor.setControl(turretPositionRequest.withPosition(targetPosition))
    }

    fun flywheelSpeed(): Double {
        return shooterLeftMotor.velocity.valueAsDouble
    }

    suspend fun waitForShooterSpeed() {
        while (true) {
            if (shooterLeftMotor.velocity.valueAsDouble >= 30 &&
                shooterLeftMotor.velocity.valueAsDouble >= shooterVelocityDutyCycleRequest.Velocity * 0.95)
                break
            delay(100.milliseconds)
        }
    }
}
