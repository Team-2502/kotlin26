package com.team2502.robot2026.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2502.robot2026.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(INTAKE_MOTOR_ID);
    private final DutyCycleOut intakeDutyCycleRequest = new DutyCycleOut(0.0);

    public Command setDutyCycle(double dutyCycle) {
        return runOnce(() -> intakeMotor.setControl(intakeDutyCycleRequest.withOutput(dutyCycle)));
    }

    public Command intake() {
        return setDutyCycle(INTAKE_IN_DUTY_CYCLE);
    }

    public Command outtake() {
        return setDutyCycle(INTAKE_OUT_DUTY_CYCLE);
    }

    public Command stop() {
        return runOnce(intakeMotor::stopMotor);
    }
}