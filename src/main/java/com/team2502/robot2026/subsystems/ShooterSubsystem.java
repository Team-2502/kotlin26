package com.team2502.robot2026.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2502.robot2026.Constants.Shooter.*;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterLeftMotor = new TalonFX(SHOOTER_LEFT_MOTOR_ID);
    private final TalonFX shooterRightMotor = new TalonFX(SHOOTER_RIGHT_MOTOR_ID);
    private final TalonFX shooterHoodMotor = new TalonFX(SHOOTER_HOOD_MOTOR_ID);

    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);
    private final PositionVoltage shooterPositionRequest = new PositionVoltage(0);

    public Command setShooterSpeed(double velocity) {
        double clampedVelocity = Math.max(SHOOTER_MIN_VELOCITY, Math.min(SHOOTER_MAX_VELOCITY, velocity));
        return runOnce(() -> {
           shooterLeftMotor.setControl(shooterVelocityRequest.withVelocity(clampedVelocity));
           shooterRightMotor.setControl(shooterVelocityRequest.withVelocity(-clampedVelocity));
        });
    }

    public Command setHoodPosition(double position) {
        double clampedPosition = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, position));
        return runOnce(() -> shooterHoodMotor.setControl(shooterPositionRequest.withPosition(clampedPosition)));
    }

    public Command stop() {
        return runOnce(() -> {
            shooterLeftMotor.stopMotor();
            shooterRightMotor.stopMotor();
            shooterHoodMotor.stopMotor();
        });
    }
}