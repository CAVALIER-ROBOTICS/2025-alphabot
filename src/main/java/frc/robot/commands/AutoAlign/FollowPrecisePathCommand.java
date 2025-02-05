// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathingConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowPrecisePathCommand extends Command {
  DriveSubsystem driveSubsystem;
  Pose2d goalPose;

  public FollowPrecisePathCommand(DriveSubsystem driveSubsystem, Pose2d goalPose) {
    this.driveSubsystem = driveSubsystem;
    this.goalPose = goalPose;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Autoaligning");
    driveSubsystem.driveToPoseWithPID(goalPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Translation2d goalTranslation = goalPose.getTranslation();
    Translation2d currentTranslation = driveSubsystem.getPoseEstimator().getPose2d().getTranslation();
    return (currentTranslation.getDistance(goalTranslation) < PathingConstants.MAXIMUM_DISTANCE_FROM_GOAL_METERS);
    // return true;
  }
}
