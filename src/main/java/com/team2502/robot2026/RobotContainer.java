// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.robot2026;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import static com.team2502.robot2026.Constants.OI.*;

public class RobotContainer {
  // Joystick mappings
  public final CommandJoystick driverLeft = new CommandJoystick(JOYSTICK_DRIVE_LEFT);
  public final CommandJoystick driverRight = new CommandJoystick(JOYSTICK_DRIVE_RIGHT);
  public final CommandJoystick operator = new CommandJoystick(JOYSTICK_OPERATOR);

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

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
