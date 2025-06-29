// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.commands.FieldDriveCommand;
import frc.robot.commands.SlowFieldDriveCommand;
import frc.robot.commands.AutoAlign.AutoScoreCommand;
import frc.robot.commands.ClimbStates.ClimbGoToJoystickSpeedCommand;
import frc.robot.commands.ClimbStates.ClimbGoToPositionCommand;
import frc.robot.commands.ElevatorStates.ElevatorGoToPositionCommand;
import frc.robot.commands.ElevatorStates.ElevatorHPIntakeCommand;
import frc.robot.commands.ElevatorStates.ElevatorRetractCommand;
import frc.robot.commands.ElevatorStates.ElevatorReturnToHomeAndZeroCommand;
import frc.robot.commands.ElevatorStates.RetractCoralAfterIntakingCommand;
import frc.robot.commands.ElevatorStates.AutonomousElevatorCommands.ExtendToHeightThenScoreCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.PathLoader;

import frc.robot.commands.ElevatorStates.ElevatorRunGrabberAndGoToPositionCommand;

public class RobotContainer { //as of 2/1/2025, we are missing two of our three subsystems. llol!
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  Command defaultDriveCommand = new FieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX);
  Command defaultElevatorCommand = new ElevatorRetractCommand(elevatorSubsystem);

  Command intakeCommand = new SequentialCommandGroup(
    new ElevatorHPIntakeCommand(elevatorSubsystem),
    new RetractCoralAfterIntakingCommand(elevatorSubsystem).withTimeout(.1)
  );

  boolean scoringOnLeft = true;
  boolean isManuallyOverrided = false;

  public RobotContainer() {
    configureNamedCommands();
    configureDefaultBindings();
    PathLoader.configureAutoBuilder(driveSubsystem, driveSubsystem.getPoseEstimator());
    configureBindings();
  }

  private void configureBindings() {
    configureElevatorBindings();
    configureDriveBindings();
    // configureClimbBindings();
    configureSideSelectorBindings();
    configureManualOverrideBindings();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("ScoreL3", new ExtendToHeightThenScoreCommand(elevatorSubsystem, ElevatorSubsystemConstants.L3_ENCODER_POSITION).withTimeout(1.5));
    NamedCommands.registerCommand("ScoreL2", new ExtendToHeightThenScoreCommand(elevatorSubsystem, ElevatorSubsystemConstants.L2_ENCODER_POSITION).withTimeout(1));
    NamedCommands.registerCommand("HPIntake", intakeCommand);
  }

  private void configureDefaultBindings() {
    driveSubsystem.setDefaultCommand(defaultDriveCommand);
    elevatorSubsystem.setDefaultCommand(defaultElevatorCommand);
    climbSubsystem.setDefaultCommand(new ClimbGoToJoystickSpeedCommand(climbSubsystem, operator::getRightY));
  }

  private void configureElevatorBindings() {
    POVButton scoreCancel = new POVButton(driver, 180);
    POVButton l2Score = new POVButton(driver, 90);
    POVButton l3Score = new POVButton(driver, 0);

    BooleanSupplier runElevatorExtruder = () -> driver.getRightTriggerAxis() > .25;
    BooleanSupplier onLeftSideBooleanSupplier = () -> getOnLeftSide();
    BooleanSupplier isOverriddenBooleanSupplier = () -> getManualOverrideStatus();

    AutoScoreCommand l2CommandAuto = new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L2_ENCODER_POSITION, onLeftSideBooleanSupplier);
    AutoScoreCommand l3CommandAuto = new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L3_ENCODER_POSITION, onLeftSideBooleanSupplier);

    ParallelCommandGroup l2CommandManual = new ParallelCommandGroup(
      new ElevatorGoToPositionCommand(elevatorSubsystem, runElevatorExtruder, ElevatorSubsystemConstants.L2_ENCODER_POSITION),
      new SlowFieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX)
    );

    ParallelCommandGroup l3CommandManual = new ParallelCommandGroup(
      new ElevatorGoToPositionCommand(elevatorSubsystem, runElevatorExtruder, ElevatorSubsystemConstants.L3_ENCODER_POSITION),
      new SlowFieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX)
    );

    l2Score.onTrue(new ConditionalCommand(l2CommandManual, l2CommandAuto, isOverriddenBooleanSupplier)); //Because the Jetson WILL catch fire during competition
    l3Score.onTrue(new ConditionalCommand(l3CommandManual, l3CommandAuto, isOverriddenBooleanSupplier));

    scoreCancel.onTrue(new ElevatorReturnToHomeAndZeroCommand(elevatorSubsystem));

    JoystickButton hpIntakeButton = new JoystickButton(driver, 6);
    hpIntakeButton.toggleOnTrue(intakeCommand);

    JoystickButton l1EjectButton = new JoystickButton(driver, 8);
    ElevatorRunGrabberAndGoToPositionCommand l1EjectCommand = new ElevatorRunGrabberAndGoToPositionCommand(elevatorSubsystem, ElevatorSubsystemConstants.HP_ENCODER_POSITION, ElevatorSubsystemConstants.INTAKE_GRABBER_SPEED); //Owen or Carter, change these zeroes to the relevant constants.
    l1EjectButton.toggleOnTrue(l1EjectCommand);
  }

  private void configureDriveBindings() {
    JoystickButton zeroDriverGyro = new JoystickButton(driver, 4);
    zeroDriverGyro.onTrue(new InstantCommand(driveSubsystem::driverGyroZero));
  }

  private void configureClimbBindings() {
    JoystickButton readyClimb = new JoystickButton(operator, 1);
    JoystickButton doClimb = new JoystickButton(operator, 2);

    readyClimb.onTrue(new ClimbGoToPositionCommand(climbSubsystem, ClimbSubsystemConstants.READY_POSITION));
    doClimb.onTrue(new ClimbGoToPositionCommand(climbSubsystem, ClimbSubsystemConstants.CLIMB_POSITION));
  }

  private void configureManualOverrideBindings() {
    JoystickButton manualOverride = new JoystickButton(operator, 8);
    manualOverride.onTrue(new InstantCommand(() -> {isManuallyOverrided = !isManuallyOverrided;}));
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

  public boolean getManualOverrideStatus() {
    return isManuallyOverrided;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
