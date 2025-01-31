// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FieldDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new FieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
