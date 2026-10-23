package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.DutyCycleOut
import com.team2502.lib.CoroutineSubsystem
import com.team2502.lib.config.create
import com.team2502.robot2026.Constants.Intake.INTAKE_IN_DUTY_CYCLE
import com.team2502.robot2026.Constants.Intake.INTAKE_OUT_DUTY_CYCLE
import com.team2502.robot2026.Constants.Intake.RAMP_IN_DUTY_CYCLE
import com.team2502.robot2026.Constants.Intake.RAMP_OUT_DUTY_CYCLE
import com.team2502.robot2026.Constants.Intake.TUNNEL_IN_DUTY_CYCLE
import com.team2502.robot2026.Constants.Intake.TUNNEL_OUT_DUTY_CYCLE
import com.team2502.robot2026.RobotCAN.INTAKE
import com.team2502.robot2026.RobotCAN.RAMP
import com.team2502.robot2026.RobotCAN.TUNNEL
import edu.wpi.first.networktables.NetworkTableInstance

class IntakeSubsystem : CoroutineSubsystem {
    private val intakeNT = NetworkTableInstance.getDefault().getTable("intakeSubsystemTable")
    private val statePub = intakeNT.getStringTopic("state").publish()
    private val dutyCycleIntakePub = intakeNT.getDoubleTopic("dutyCycleIntake").publish()
    private val dutyCycleTunnelPub = intakeNT.getDoubleTopic("dutyCycleTunnel").publish()
    private val dutyCycleRampPub = intakeNT.getDoubleTopic("dutyCycleRamp").publish()

    private val intakeMotor = INTAKE.create()
    private val tunnelMotor = TUNNEL.create()
    private val rampMotor = RAMP.create()

    private var state = "INITIALIZED"
    private val intakeDutyCycleRequest = DutyCycleOut(0.0)
    private val tunnelDutyCycleRequest = DutyCycleOut(0.0)
    private val rampDutyCycleRequest = DutyCycleOut(0.0)

    private fun setDutyCycle(dutyCycleIntake: Double, dutyCycleTunnel: Double, dutyCycleRamp: Double) {
        intakeMotor.setControl(intakeDutyCycleRequest.withOutput(dutyCycleIntake))
        tunnelMotor.setControl(tunnelDutyCycleRequest.withOutput(dutyCycleTunnel))
        rampMotor.setControl(rampDutyCycleRequest.withOutput(dutyCycleRamp))
    }

    override fun periodic() {
        statePub.set(state)
        dutyCycleIntakePub.set(intakeDutyCycleRequest.Output)
        dutyCycleTunnelPub.set(tunnelDutyCycleRequest.Output)
        dutyCycleRampPub.set(rampDutyCycleRequest.Output)
    }

    override fun stop() {
        state = "STOPPED"

        intakeMotor.stopMotor()
        tunnelMotor.stopMotor()
        rampMotor.stopMotor()
    }

    fun intake() {
        state = "INTAKE"
        setDutyCycle(
            INTAKE_IN_DUTY_CYCLE,
            TUNNEL_IN_DUTY_CYCLE,
            RAMP_IN_DUTY_CYCLE
        )
    }

    fun outtake() {
        state = "OUTTAKE"
        setDutyCycle(
            INTAKE_OUT_DUTY_CYCLE,
            TUNNEL_OUT_DUTY_CYCLE,
            RAMP_OUT_DUTY_CYCLE
        )
    }
}