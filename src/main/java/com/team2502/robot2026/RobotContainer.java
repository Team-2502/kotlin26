// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.robot2026;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team2502.robot2026.subsystems.IntakeSubsystem;
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain;
import com.team2502.robot2026.subsystems.drive.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import static com.team2502.robot2026.Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static com.team2502.robot2026.Constants.Drivetrain.MEASURED_MAX_SPEED_METERS_PER_SECOND;
import static com.team2502.robot2026.Constants.OI.*;

public class RobotContainer {
  // Joystick mappings
  public final CommandJoystick driverLeft = new CommandJoystick(JOYSTICK_DRIVE_LEFT);
  public final CommandJoystick driverRight = new CommandJoystick(JOYSTICK_DRIVE_RIGHT);
  public final CommandJoystick operator = new CommandJoystick(JOYSTICK_OPERATOR);

  // Subsystems
  public final CommandSwerveDrivetrain drivetrainSubsystem = TunerConstants.createDrivetrain();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Swerve Command Setups
  private final SwerveRequest.FieldCentric driveRequest =
          new SwerveRequest
                  .FieldCentric()
                  .withDeadband(JOYSTICK_DEADBAND)
                  .withRotationalDeadband(JOYSTICK_ROTATION_DEADBAND)
                  .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Drivetrain bindings
    drivetrainSubsystem.setDefaultCommand(
            drivetrainSubsystem.applyRequest(() -> driveRequest
                    .withVelocityX(-driverLeft.getX() * MEASURED_MAX_SPEED_METERS_PER_SECOND) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverLeft.getY() * MEASURED_MAX_SPEED_METERS_PER_SECOND) // Drive left with negative X (left)
                    .withRotationalRate(-driverRight.getZ() * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) // Drive counterclockwise with negative Z (CCW Turn))
    ));

    driverRight.trigger().whileTrue(intakeSubsystem.intake()).whileFalse(intakeSubsystem.stop());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
