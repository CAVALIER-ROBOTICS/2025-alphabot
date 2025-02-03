// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathingConstants;
import frc.robot.commands.ElevatorStates.ElevatorRetractCommand;
import frc.robot.commands.ElevatorStates.AutonomousElevatorCommands.ExtendToHeightThenScoreCommand;
import frc.robot.subsystems.ElevatorSubsystem;

/** Add your docs here. */
public class AutoAlignCommandFactory {
    static List<Pose2d> redAllianceScoringPositions = new ArrayList<>();
    static boolean redAllianceScoringPositionsInitialized = false;

    public static void initRedAllianceScoringPositions() {
        for(Pose2d pose: PathingConstants.BLUE_SIDED_SCORING_POSITIONS) {
            Pose2d poseToAdd = new Pose2d(PathingConstants.FIELD_WIDTH_METERS - pose.getX(), pose.getY(), pose.getRotation());
            redAllianceScoringPositions.add(poseToAdd);
        }
    }

    private static void checkRedAllianceInitialized() {
        if(!redAllianceScoringPositionsInitialized) {
            redAllianceScoringPositionsInitialized = true;
            initRedAllianceScoringPositions();
        }
    }

    public static Pose2d getClosestPose(Pose2d origin, boolean onRedAlliance) {
        List<Pose2d> poseList = PathingConstants.BLUE_SIDED_SCORING_POSITIONS;
        if(onRedAlliance) {
            poseList = redAllianceScoringPositions;
        }
        return origin.nearest(poseList);
    }

    public static Command getAutoAlignDriveCommand(Pose2d currentPosition, boolean onRedAlliance) {
        checkRedAllianceInitialized();
        Pose2d goalPose = getClosestPose(currentPosition, onRedAlliance);
        return PathLoader.pathfindToPose(goalPose);
    }

    public static Command getAutoAlignAndScoreCommand(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, double elevatorEncoderPosition, boolean onRedAlliance) {
        return new SequentialCommandGroup(
            getAutoAlignDriveCommand(currentPosition, onRedAlliance),
            new ExtendToHeightThenScoreCommand(elevatorSubsystem, elevatorEncoderPosition),
            new ElevatorRetractCommand(elevatorSubsystem)
        );
    }
}
