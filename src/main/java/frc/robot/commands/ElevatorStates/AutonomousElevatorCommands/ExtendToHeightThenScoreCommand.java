// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorStates.AutonomousElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ExtendToHeightThenScoreCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  double positionSetpoint;

  public ExtendToHeightThenScoreCommand(ElevatorSubsystem elevatorSubsystem, double positionSetpoint) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.positionSetpoint = positionSetpoint;

    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setPosition(positionSetpoint);

    double grabberSpeed = (elevatorSubsystem.isElevatorPIDAtSetpoint()) ? ElevatorSubsystemConstants.GRABBER_SPEED: 0;
    elevatorSubsystem.setGrabber(grabberSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getIsCoralInHoldingPosition();
  }
}
