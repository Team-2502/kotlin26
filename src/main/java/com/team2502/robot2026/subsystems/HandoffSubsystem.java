package com.team2502.robot2026.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2502.robot2026.Constants.Handoff.*;

public class HandoffSubsystem extends SubsystemBase {
    private final TalonFX tunnelMotor = new TalonFX(TUNNEL_MOTOR_ID);
    private final TalonFX rampMotor = new TalonFX(RAMP_MOTOR_ID);
    private final DutyCycleOut handoffDutyCycleRequest = new DutyCycleOut(0.0);

    public Command setTunnelDutyCycle(double dutyCycle) {
        return runOnce(() -> tunnelMotor.setControl(handoffDutyCycleRequest.withOutput(dutyCycle)));
    }

    public Command setRampDutyCycle(double dutyCycle) {
        return runOnce(() -> rampMotor.setControl(handoffDutyCycleRequest.withOutput(dutyCycle)));
    }

    public Command intake() {
        return setRampDutyCycle(HANDOFF_RAMP_IN_DUTY_CYCLE)
                .alongWith(setTunnelDutyCycle(HANDOFF_TUNNEL_IN_DUTY_CYCLE));
    }

    public Command outtake() {
        return setRampDutyCycle(HANDOFF_RAMP_OUT_DUTY_CYCLE)
                .alongWith(setTunnelDutyCycle(HANDOFF_TUNNEL_OUT_DUTY_CYCLE));
    }

    public Command stop() {
        return runOnce(() -> {
            tunnelMotor.stopMotor();
            rampMotor.stopMotor();
        });
    }
}