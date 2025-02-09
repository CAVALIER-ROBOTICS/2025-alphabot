// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.commands.FieldDriveCommand;
import frc.robot.commands.AutoAlign.AutoScoreCommand;
import frc.robot.commands.ElevatorStates.ElevatorGoToPositionCommand;
import frc.robot.commands.ElevatorStates.ElevatorHPIntakeCommand;
import frc.robot.commands.ElevatorStates.ElevatorRetractCommand;
import frc.robot.commands.ElevatorStates.ElevatorReturnToHomeAndZeroCommand;
import frc.robot.commands.ElevatorStates.RetractCoralAfterIntakingCommand;
import frc.robot.commands.ElevatorStates.AutonomousElevatorCommands.ExtendToHeightThenScoreCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.PathLoader;

public class RobotContainer { //as of 2/1/2025, we are missing two of our three subsystems. llol!
  XboxController driver = new XboxController(0);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  Command defaultDriveCommand = new FieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX);
  Command defaultElevatorCommand = new ElevatorRetractCommand(elevatorSubsystem);

  Command intakeCommand = new SequentialCommandGroup(
    new ElevatorHPIntakeCommand(elevatorSubsystem),
    new RetractCoralAfterIntakingCommand(elevatorSubsystem).withTimeout(.1)
  );

  Command zeroElevatorAndIntakeCommand = new SequentialCommandGroup(
    new ElevatorReturnToHomeAndZeroCommand(elevatorSubsystem),
    intakeCommand
  );

  boolean scoringOnLeft = true;

  public RobotContainer() {
    configureNamedCommands();
    configureDefaultBindings();
    PathLoader.configureAutoBuilder(driveSubsystem, driveSubsystem.getPoseEstimator());
    configureBindings();
  }

  private void configureBindings() {
    configureElevatorBindings();
    configureDriveBindings();
    configureSideSelectorBindings();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("ScoreL3", new ExtendToHeightThenScoreCommand(elevatorSubsystem, driveSubsystem, ElevatorSubsystemConstants.L3_ENCODER_POSITION).withTimeout(1.5));
    NamedCommands.registerCommand("ScoreL2", new ExtendToHeightThenScoreCommand(elevatorSubsystem, driveSubsystem, ElevatorSubsystemConstants.L2_ENCODER_POSITION).withTimeout(1));

    NamedCommands.registerCommand("HPIntake", intakeCommand);
  }

  private void configureDefaultBindings() {
    driveSubsystem.setDefaultCommand(defaultDriveCommand);
    elevatorSubsystem.setDefaultCommand(defaultElevatorCommand);
  }

  private void configureElevatorBindings() {
    POVButton l1Score = new POVButton(driver, 180);
    POVButton l2Score = new POVButton(driver, 90);
    POVButton l3Score = new POVButton(driver, 0);

    // BooleanSupplier runElevatorExtruder = () -> driver.getRightTriggerAxis() > .25;
    // l2Score.onTrue(new ElevatorGoToPositionCommand(elevatorSubsystem, runElevatorExtruder, ElevatorSubsystemConstants.L2_ENCODER_POSITION));
    // l3Score.onTrue(new ElevatorGoToPositionCommand(elevatorSubsystem, runElevatorExtruder, ElevatorSubsystemConstants.L3_ENCODER_POSITION));
    // scoreCancel.onTrue(new ElevatorReturnToHomeAndZeroCommand(elevatorSubsystem));

    //with autoscoring
    BooleanSupplier onLeftSideBooleanSupplier = () -> getOnLeftSide();
    l1Score.onTrue(new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L1_ENCODER_POSITION, onLeftSideBooleanSupplier, ElevatorSubsystemConstants.L1_GRABBER_SPEED));
    l2Score.onTrue(new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L2_ENCODER_POSITION, onLeftSideBooleanSupplier));
    l3Score.onTrue(new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L3_ENCODER_POSITION, onLeftSideBooleanSupplier));

    JoystickButton hpIntakeButton = new JoystickButton(driver, 6);
    hpIntakeButton.toggleOnTrue(zeroElevatorAndIntakeCommand);
  }

  private void configureDriveBindings() {
    JoystickButton zeroDriverGyro = new JoystickButton(driver, 4);
    zeroDriverGyro.onTrue(new InstantCommand(driveSubsystem::driverGyroZero));
  }

  private void configureSideSelectorBindings() {
    JoystickButton leftSelector = new JoystickButton(driver, 3);
    JoystickButton rightSelector = new JoystickButton(driver, 2);

    leftSelector.onTrue(new InstantCommand(() -> {
      System.out.println("Left selector pressed");
      scoringOnLeft = true;
    }));
    rightSelector.onTrue(new InstantCommand(() -> {scoringOnLeft = false;}));
  }

  public boolean getOnLeftSide() {
    return scoringOnLeft;
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
