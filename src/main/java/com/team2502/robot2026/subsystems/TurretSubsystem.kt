package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.team2502.lib.FileZero
import com.team2502.lib.config.create
import com.team2502.robot2026.Constants.Turret.ABS_TO_RELATIVE_RATIO
import com.team2502.robot2026.Constants.Turret.MOTOR_TO_TURRET_RATIO
import com.team2502.robot2026.Constants.Turret.TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS
import com.team2502.robot2026.Constants.Turret.TURRET_CLAMP
import com.team2502.robot2026.RobotCAN.TURRET
import com.team2502.robot2026.RobotCAN.TURRET_ENCODER
import edu.wpi.first.math.MathUtil.angleModulus
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.io.File

class TurretSubsystem : SubsystemBase() {
    private val turretMotor = TURRET.create()
    private val turretEncoder = TURRET_ENCODER.create()

    private val turretPositionRequest = PositionVoltage(0.0)

    var turretZeroPosition: Rotation2d? = null

    fun initialize() {
        turretZeroPosition = Rotation2d.fromRotations(FileZero(
            File("/tmp/turret_zero"),
            {
                turretMotor.position.valueAsDouble +
                        (turretEncoder.absolutePosition.valueAsDouble - TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS) *
                        ABS_TO_RELATIVE_RATIO
            }
        ))
    }

    private fun setTurretPosition(position: Double) {
        turretMotor.setControl(turretPositionRequest.withPosition(position))
    }

    /** points turret to a ROBOT relative angle, with 0 being away from intake **/
    fun pointTo(angle: Rotation2d) {
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
        setTurretPosition(targetPosition)
    }
}
