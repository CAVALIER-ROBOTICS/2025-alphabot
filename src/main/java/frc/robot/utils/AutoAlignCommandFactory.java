// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathingConstants;
import frc.robot.commands.AutoAlign.FollowPrecisePathCommand;
import frc.robot.commands.ElevatorStates.ElevatorRetractCommand;
import frc.robot.commands.ElevatorStates.AutonomousElevatorCommands.ExtendToHeightThenScoreCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/** Add your docs here. */
public class AutoAlignCommandFactory {
    static List<Pose2d> redAllianceScoringPositions = new ArrayList<>();
    static List<Pose2d> blueAllianceScoringPositions = new ArrayList<>();

    static boolean initialized = false;

    public static void initRedAllianceScoringPositions() {
        for(Pose2d pose: blueAllianceScoringPositions) {
            Pose2d poseToAdd = new Pose2d(PathingConstants.FIELD_WIDTH_METERS - pose.getX(), PathingConstants.FIELD_HEIGHT_METERS - pose.getY(), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
            redAllianceScoringPositions.add(poseToAdd);
        }
    }

    public static void applyXYOffsetsToBlueAlliance(double x, double y) {
        List<Pose2d> originalPoseList = PathingConstants.BLUE_SIDED_SCORING_POSITIONS;
        for(int i = 0; i < PathingConstants.BLUE_SIDED_SCORING_POSITIONS.size(); i++) {
            // Field2d field = new Field2d();
            Pose2d originalPose = originalPoseList.get(i);
            double offset = Math.atan2(x, y);
            double magOffset = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            double poseOrientationRadians = originalPose.getRotation().getRadians();

            double newX = originalPose.getX() + ((Math.cos(poseOrientationRadians + offset) * magOffset));
            double newY = originalPose.getY() + ((Math.sin(poseOrientationRadians + offset) * magOffset));

            Pose2d np = new Pose2d(newX, newY, originalPose.getRotation());

            // field.setRobotPose(np);
            // SmartDashboard.putData(String.valueOf(i), field);

            blueAllianceScoringPositions.add(np);
        }
    }

    private static void initalize() {
        if(!initialized) {
            initialized = true;
            applyXYOffsetsToBlueAlliance(PathingConstants.X_OFFSET, PathingConstants.Y_OFFSET);
            initRedAllianceScoringPositions();
        }
    }

    public static Pose2d getClosestPose(Pose2d origin, boolean onRedAlliance) {
        initalize();
        List<Pose2d> poseList = blueAllianceScoringPositions;
        if(onRedAlliance) {
            poseList = redAllianceScoringPositions;
        }
        return origin.nearest(poseList);
    }

    public static Command getAutoAlignDriveCommand(DriveSubsystem driveSubsystem, Pose2d currentPosition, boolean onRedAlliance) {
        initalize();
        Pose2d goalPose = getClosestPose(currentPosition, onRedAlliance);

        return new SequentialCommandGroup(
            new FollowPrecisePathCommand(driveSubsystem, goalPose)
        );
    }

    public static Command getAutoAlignAndScoreCommand(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, double elevatorEncoderPosition, boolean onRedAlliance) {
        return new SequentialCommandGroup(
            getAutoAlignDriveCommand(driveSubsystem, currentPosition, onRedAlliance),
            new ExtendToHeightThenScoreCommand(elevatorSubsystem, driveSubsystem, elevatorEncoderPosition),
            new ElevatorRetractCommand(elevatorSubsystem)
        );
    }
}
