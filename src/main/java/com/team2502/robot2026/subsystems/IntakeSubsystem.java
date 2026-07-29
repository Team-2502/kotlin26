package com.team2502.robot2026.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2502.robot2026.Constants.Intake.INTAKE_MOTOR_ID;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final DutyCycleOut intakeDutyCycle;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(INTAKE_MOTOR_ID);
        intakeDutyCycle = new DutyCycleOut(0.0);
    }

    public Command intake() {
        return new InstantCommand(() -> intakeMotor.setControl(intakeDutyCycle.withOutput(1.0)));
    }

    public Command stop() {
        return new InstantCommand(() -> intakeMotor.setControl(intakeDutyCycle.withOutput(0.0)));
    }
}