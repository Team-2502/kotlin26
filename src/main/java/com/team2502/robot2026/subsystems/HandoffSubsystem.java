package com.team2502.robot2026.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2502.robot2026.Constants.Handoff.*;
import static com.team2502.robot2026.Constants.Intake.INTAKE_IN_DUTY_CYCLE;
import static com.team2502.robot2026.Constants.Intake.INTAKE_OUT_DUTY_CYCLE;
import static edu.wpi.first.hal.simulation.DigitalPWMDataJNI.setDutyCycle;

public class HandoffSubsystem extends SubsystemBase {
    private final TalonFX tunnelMotor;
    private final TalonFX rampMotor;

    private final DutyCycleOut handoffDutyCycle;

    public HandoffSubsystem() {
        tunnelMotor = new TalonFX(TUNNEL_MOTOR_ID);
        rampMotor = new TalonFX(RAMP_MOTOR_ID);

        handoffDutyCycle = new DutyCycleOut(0.0);
    }

    public Command setTunnelDutyCycle(double dutyCycle) {
        return new InstantCommand(
                () -> tunnelMotor.setControl(handoffDutyCycle.withOutput(dutyCycle))
        );
    }

    public Command setRampDutyCycle(double dutyCycle) {
        return new InstantCommand(
                () -> rampMotor.setControl(handoffDutyCycle.withOutput(dutyCycle))
        );
    }

    public Command intake() {
        return new ParallelCommandGroup(
                setRampDutyCycle(HANDOFF_RAMP_IN_DUTY_CYCLE),
                setTunnelDutyCycle(HANDOFF_TUNNEL_IN_DUTY_CYCLE)
        );
    }

    public Command outtake() {
        return new ParallelCommandGroup(
                setRampDutyCycle(HANDOFF_RAMP_OUT_DUTY_CYCLE),
                setTunnelDutyCycle(HANDOFF_TUNNEL_OUT_DUTY_CYCLE)
        );
    }

    public Command stop() {
        return new ParallelCommandGroup(
                setRampDutyCycle(0.0),
                setTunnelDutyCycle(0.0)
        );
    }
}