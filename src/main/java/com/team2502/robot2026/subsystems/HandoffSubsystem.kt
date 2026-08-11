package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.TalonFX
import com.team2502.robot2026.Constants.Handoff
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class HandoffSubsystem : SubsystemBase() {
    private val tunnelMotor = TalonFX(Handoff.TUNNEL_MOTOR_ID)
    private val rampMotor = TalonFX(Handoff.RAMP_MOTOR_ID)
    private val handoffDutyCycleRequest = DutyCycleOut(0.0)

    fun setTunnelDutyCycle(dutyCycle: Double): Command {
        return runOnce { tunnelMotor.setControl(handoffDutyCycleRequest.withOutput(dutyCycle)) }
    }

    fun setRampDutyCycle(dutyCycle: Double): Command {
        return runOnce { rampMotor.setControl(handoffDutyCycleRequest.withOutput(dutyCycle)) }
    }

    fun intake(): Command {
        return setRampDutyCycle(Handoff.HANDOFF_RAMP_IN_DUTY_CYCLE)
            .alongWith(setTunnelDutyCycle(Handoff.HANDOFF_TUNNEL_IN_DUTY_CYCLE))
    }

    fun outtake(): Command {
        return setRampDutyCycle(Handoff.HANDOFF_RAMP_OUT_DUTY_CYCLE)
            .alongWith(setTunnelDutyCycle(Handoff.HANDOFF_TUNNEL_OUT_DUTY_CYCLE))
    }

    fun stop(): Command {
        return runOnce {
            tunnelMotor.stopMotor()
            rampMotor.stopMotor()
        }
    }
}