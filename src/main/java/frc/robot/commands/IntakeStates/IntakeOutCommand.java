// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeStates;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  public IntakeOutCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setPosition(IntakeSubsystemConstants.INTAKE_OUT_POS);
    intakeSubsystem.setIntakeSpin(IntakeSubsystemConstants.INTAKE_SPIN_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intakeSubsystem.getFlywheelCurrentDraw() > IntakeSubsystemConstants.SUCCESSFUL_BALL_INTAKE_CURRENT_DRAW);
  }
}
