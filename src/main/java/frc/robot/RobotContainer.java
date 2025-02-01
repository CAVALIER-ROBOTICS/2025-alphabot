// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.commands.FieldDriveCommand;
import frc.robot.commands.ElevatorStates.ElevatorGoToPositionCommand;
import frc.robot.commands.ElevatorStates.ElevatorRetractCommand;
import frc.robot.commands.IntakeStates.IntakeInCommand;
import frc.robot.commands.IntakeStates.IntakeOutCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.PathLoader;

public class RobotContainer {
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  public RobotContainer() {
    configureDefaultBindings();
    PathLoader.configureAutoBuilder(driveSubsystem, driveSubsystem.getPoseEstimator());
    configureBindings();
  }

  private void configureBindings() {
    configureElevatorBindings();
    configureDriveBindings();
    configureIntakeBindings();
  }

  private void configureDefaultBindings() {
    driveSubsystem.setDefaultCommand(new FieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX));
    intakeSubsystem.setDefaultCommand(new IntakeInCommand(intakeSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  private void configureElevatorBindings() {
    POVButton l2Score = new POVButton(driver, 90);
    POVButton l3Score = new POVButton(driver, 0);
    POVButton scoreCancel = new POVButton(driver, 180);

    BooleanSupplier runElevatorExtruder = () -> driver.getLeftTriggerAxis() > .25;
    l2Score.onTrue(new ElevatorGoToPositionCommand(elevatorSubsystem, runElevatorExtruder, ElevatorSubsystemConstants.L2_ENCODER_POSITION));
    l3Score.onTrue(new ElevatorGoToPositionCommand(elevatorSubsystem, runElevatorExtruder, ElevatorSubsystemConstants.L3_ENCODER_POSITION));
    scoreCancel.onTrue(new ElevatorRetractCommand(elevatorSubsystem));
  }

  private void configureDriveBindings() {
    JoystickButton zeroDriverGyro = new JoystickButton(driver, 4);
    zeroDriverGyro.onTrue(new InstantCommand(driveSubsystem::driverGyroZero));
  }

  private void configureIntakeBindings() {
    JoystickButton intakeOut = new JoystickButton(driver, 5);
    intakeOut.toggleOnTrue(new IntakeOutCommand(intakeSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
