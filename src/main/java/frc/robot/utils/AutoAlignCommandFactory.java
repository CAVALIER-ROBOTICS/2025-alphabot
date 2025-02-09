// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathingConstants;
import frc.robot.commands.AutoAlign.FollowPrecisePathCommand;
import frc.robot.commands.ElevatorStates.AutonomousElevatorCommands.ExtendToHeightThenScoreCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/** Add your docs here. */
public class AutoAlignCommandFactory {
    static List<Pose2d> leftBlueAllianceScoringPositions = new ArrayList<>();
    static List<Pose2d> rightBlueAllianceScoringPositions = new ArrayList<>();

    static List<Pose2d> rightRedAllianceScoringPositions = new ArrayList<>();
    static List<Pose2d> leftRedAllianceScoringPositions = new ArrayList<>();

    static boolean initialized = false;

    public static List<Pose2d> mirrorBlueSidedPoseList(List<Pose2d> list) {
        List<Pose2d> ret = new ArrayList<>();
        for(Pose2d pose: list) {
            Pose2d poseToAdd = new Pose2d(PathingConstants.FIELD_WIDTH_METERS - pose.getX(), PathingConstants.FIELD_HEIGHT_METERS - pose.getY(), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
            ret.add(poseToAdd);
        }
        return ret;
    }

    @SuppressWarnings("unused")
    private static void display(List<Pose2d> list) {
        for(int i = 0; i < list.size(); i++) {
            Field2d field = new Field2d();
            field.setRobotPose(list.get(i));
            SmartDashboard.putData(String.valueOf(i), field);
        }
    }

    public static List<Pose2d> applyXYOffsetsToPoseList(double x, double y, List<Pose2d> originalPoseList) {
        List<Pose2d> ret = new ArrayList<>();
        for(int i = 0; i < originalPoseList.size(); i++) {
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

            ret.add(np);
        }
        return ret;
    }

    public static void initalize() {
        if(!initialized) {
            initialized = true;
            leftBlueAllianceScoringPositions = applyXYOffsetsToPoseList(PathingConstants.X_OFFSET, PathingConstants.Y_OFFSET, PathingConstants.LEFT_BLUE_SIDED_SCORING_POSITIONS);
            rightBlueAllianceScoringPositions = applyXYOffsetsToPoseList(PathingConstants.X_OFFSET, PathingConstants.Y_OFFSET, PathingConstants.RIGHT_BLUE_SIDED_SCORING_POSITIONS);

            leftRedAllianceScoringPositions = mirrorBlueSidedPoseList(leftBlueAllianceScoringPositions);
            rightRedAllianceScoringPositions = mirrorBlueSidedPoseList(rightBlueAllianceScoringPositions);
        }
    }

    public static Pose2d getClosestPose(Pose2d origin, boolean onRedAlliance, boolean left) {
        initalize();
        List<Pose2d> poseList;

        if(left) { //I'm sorry
            if(onRedAlliance) {
                poseList = leftRedAllianceScoringPositions;
            } else {
                poseList = leftBlueAllianceScoringPositions;
            }
        } else {
            if(onRedAlliance) {
                poseList = rightRedAllianceScoringPositions;
            } else {
                poseList = rightBlueAllianceScoringPositions;
            }
        }

        return origin.nearest(poseList);
    }

    public static Command getAutoAlignDriveCommand(DriveSubsystem driveSubsystem, Pose2d currentPosition, boolean onRedAlliance, boolean onLeftSide) {
        initalize();
        Pose2d goalPose = getClosestPose(currentPosition, onRedAlliance, onLeftSide);

        return new SequentialCommandGroup(
            new FollowPrecisePathCommand(driveSubsystem, goalPose)
        );
    }

    public static Command getAutoAlignAndScoreCommand(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, double elevatorEncoderPosition, boolean onRedAlliance, boolean onLeftSide) {
        System.out.println(onLeftSide);
        return new SequentialCommandGroup(
            getAutoAlignDriveCommand(driveSubsystem, currentPosition, onRedAlliance, onLeftSide),
            new ExtendToHeightThenScoreCommand(elevatorSubsystem, driveSubsystem, elevatorEncoderPosition)
        );
    }

    public static Command getAutoAlignAndScoreCommand(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, double elevatorEncoderPosition, boolean onRedAlliance, boolean onLeftSide, double grabberSpeed) {
        System.out.println(onLeftSide);
        return new SequentialCommandGroup(
            getAutoAlignDriveCommand(driveSubsystem, currentPosition, onRedAlliance, onLeftSide),
            new ExtendToHeightThenScoreCommand(elevatorSubsystem, driveSubsystem, elevatorEncoderPosition, grabberSpeed)
        );
    }
}
