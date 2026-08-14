package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.team2502.lib.config.create
import com.team2502.robot2026.Constants.Turret.ABS_TO_RELATIVE_RATIO
import com.team2502.robot2026.Constants.Turret.MOTOR_TO_TURRET_RATIO
import com.team2502.robot2026.Constants.Turret.TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS
import com.team2502.robot2026.Constants.Turret.TURRET_CLAMP
import com.team2502.robot2026.RobotCAN.TURRET
import com.team2502.robot2026.RobotCAN.TURRET_ENCODER
import edu.wpi.first.math.MathUtil.angleModulus
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.io.File
import java.io.IOException

class TurretSubsystem : SubsystemBase() {
    private val turretMotor = TURRET.create()
    private val turretEncoder = TURRET_ENCODER.create()

    private val turretPositionRequest = PositionVoltage(0.0)

    private val turretZeroPosition: Rotation2d

    init {
        val turretZeroFromFile = File("/tmp/turret_zero").run {
            if (!exists())
                return@run null
            val content = try {
                readText()
            } catch (e: IOException) {
                return@run null
            }
            return@run Rotation2d.fromRotations(content.toDouble())
        }

        turretZeroPosition = turretZeroFromFile ?:
            Rotation2d(
                turretMotor.position.valueAsDouble +
                        (turretEncoder.absolutePosition.valueAsDouble - TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS) *
                        ABS_TO_RELATIVE_RATIO
            )

        if (turretZeroFromFile == null) {
            File("/tmp/turret_zero").writeText(turretZeroPosition.rotations.toString())
        }
    }

    private fun setTurretPosition(position: Double): Command {
        return runOnce({turretMotor.setControl(turretPositionRequest.withPosition(position))})
    }

    /** points turret to a ROBOT relative angle, with 0 being away from intake **/
    fun pointTo(angle: Rotation2d): Command {
        val currentPositon = turretMotor.position.valueAsDouble

        val wrappedAngle = Rotation2d(angleModulus(angle.radians))
        val targetPosition =
            ((wrappedAngle.rotations * MOTOR_TO_TURRET_RATIO) + turretZeroPosition.rotations)
                .coerceIn(currentPositon - TURRET_CLAMP, currentPositon + TURRET_CLAMP)
        return runOnce({setTurretPosition(targetPosition)})
    }
}
