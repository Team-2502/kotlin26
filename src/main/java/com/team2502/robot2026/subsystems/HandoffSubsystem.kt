package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.TalonFX
import com.team2502.robot2026.Constants.Handoff
import com.team2502.robot2026.RobotCAN.RAMP
import com.team2502.robot2026.RobotCAN.TUNNEL
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class HandoffSubsystem : SubsystemBase() {
    private val tunnelMotor = TalonFX(TUNNEL.canId)
    private val rampMotor = TalonFX(RAMP.canId)
    private val handoffDutyCycleRequest = DutyCycleOut(0.0)

    init {
        tunnelMotor.configurator.apply(TUNNEL.config.generate())
        rampMotor.configurator.apply(RAMP.config.generate())
    }

    fun setTunnelDutyCycle(dutyCycle: Double): Command {
        return runEnd(
            {tunnelMotor.setControl(handoffDutyCycleRequest.withOutput(dutyCycle))},
            {tunnelMotor.setControl(handoffDutyCycleRequest.withOutput(0.0))}
        )
    }

    fun setRampDutyCycle(dutyCycle: Double): Command {
        return runEnd(
            {rampMotor.setControl(handoffDutyCycleRequest.withOutput(dutyCycle))},
            {rampMotor.setControl(handoffDutyCycleRequest.withOutput(0.0))}
        )
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