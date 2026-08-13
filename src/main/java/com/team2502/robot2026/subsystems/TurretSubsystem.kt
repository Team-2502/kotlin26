package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.team2502.robot2026.Constants.Turret.MOTOR_TO_TURRET_RATIO
import com.team2502.robot2026.Constants.Turret.TURRET_CLAMP
import com.team2502.robot2026.RobotCAN.TURRET
import edu.wpi.first.math.MathUtil.angleModulus
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class TurretSubsystem : SubsystemBase() {
    private val turretMotor = TalonFX(TURRET.canId)
    private val turretPositionRequest = PositionVoltage(0.0)

    private val turretZeroPosition: Rotation2d

    init {
        turretMotor.configurator.apply(TURRET.config.generate())

        turretZeroPosition = Rotation2d(0.0)
    }

    private fun setTurretPosition(position: Double): Command {
        return runOnce({turretMotor.setControl(turretPositionRequest.withPosition(position))})
    }

    /** points turret to a ROBOT relative angle, with 0 being away from intake **/
    fun pointTo(angle: Rotation2d): Command {
        val currentPositon = turretMotor.position.valueAsDouble

        // turret zero position hardcoded as of 08/12/2026
        val wrappedAngle = Rotation2d(angleModulus(angle.radians))
        val targetPosition =
            ((wrappedAngle.rotations * MOTOR_TO_TURRET_RATIO) + turretZeroPosition.rotations)
                .coerceIn(currentPositon - TURRET_CLAMP, currentPositon + TURRET_CLAMP)
        return runOnce({setTurretPosition(targetPosition)})
    }
}
