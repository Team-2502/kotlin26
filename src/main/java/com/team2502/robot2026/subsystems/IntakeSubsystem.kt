package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.TalonFX
import com.team2502.robot2026.Constants
import com.team2502.robot2026.RobotCAN.INTAKE
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class IntakeSubsystem : SubsystemBase() {
    private val intakeMotor= TalonFX(Constants.Intake.INTAKE_MOTOR_ID)
    private val intakeDutyCycleRequest = DutyCycleOut(0.0)

    init {
        intakeMotor.configurator.apply(INTAKE.config.generate())
    }

    fun setDutyCycle(dutyCycle: Double): Command {
        return runOnce { intakeMotor.setControl(intakeDutyCycleRequest.withOutput(dutyCycle)) }
    }

    fun intake(): Command {
        return setDutyCycle(Constants.Intake.INTAKE_IN_DUTY_CYCLE)
    }

    fun outtake(): Command {
        return setDutyCycle(Constants.Intake.INTAKE_OUT_DUTY_CYCLE)
    }

    fun stop(): Command {
        return runOnce { intakeMotor.stopMotor() }
    }
}