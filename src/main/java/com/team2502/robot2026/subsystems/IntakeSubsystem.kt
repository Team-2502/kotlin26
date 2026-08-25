package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.DutyCycleOut
import com.team2502.lib.config.create
import com.team2502.robot2026.Constants
import com.team2502.robot2026.Constants.Intake.INTAKE_IN_DUTY_CYCLE
import com.team2502.robot2026.Constants.Intake.INTAKE_OUT_DUTY_CYCLE
import com.team2502.robot2026.Constants.Intake.TUNNEL_IN_DUTY_CYCLE
import com.team2502.robot2026.Constants.Intake.TUNNEL_OUT_DUTY_CYCLE
import com.team2502.robot2026.RobotCAN.INTAKE
import com.team2502.robot2026.RobotCAN.TUNNEL
import edu.wpi.first.wpilibj2.command.SubsystemBase

class IntakeSubsystem : SubsystemBase() {
    private val intakeMotor = INTAKE.create()
    private val tunnelMotor = TUNNEL.create()
    private val intakeDutyCycleRequest = DutyCycleOut(0.0)

    private fun setDutyCycle(dutyCycleIntake: Double, dutyCycleTunnel: Double) {
        intakeMotor.setControl(intakeDutyCycleRequest.withOutput(dutyCycleIntake))
        tunnelMotor.setControl(intakeDutyCycleRequest.withOutput(dutyCycleTunnel))
    }

    fun intake() {
        setDutyCycle(INTAKE_IN_DUTY_CYCLE, TUNNEL_IN_DUTY_CYCLE)
    }

    fun outtake() {
        setDutyCycle(INTAKE_OUT_DUTY_CYCLE, TUNNEL_OUT_DUTY_CYCLE)
    }

    fun stop() {
        intakeMotor.stopMotor()
        tunnelMotor.stopMotor()
    }
}