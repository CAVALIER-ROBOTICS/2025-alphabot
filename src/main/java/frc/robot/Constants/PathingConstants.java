package frc.robot.Constants;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathingConstants {
    //find actual values
    public static final PathConstraints PATHFINDING_CONSTRAINTS = new PathConstraints(
        3.0, 
        4.0, 
        Math.toRadians(540), 
        Math.toRadians(270)
    );

    public static final List<Pose2d> BLUE_SIDED_SCORING_POSITIONS = Arrays.asList( //gotta mirror em
        new Pose2d(3.826, 5.174, Rotation2d.fromDegrees(-60)),
        new Pose2d(4.108, 5.335, Rotation2d.fromDegrees(-60)),
        new Pose2d(3.163, 4.356, Rotation2d.fromDegrees(0.0)),
        new Pose2d(3.163, 4.031, Rotation2d.fromDegrees(0.0)),
        new Pose2d(3.533, 3.047, Rotation2d.fromDegrees(60)),
        new Pose2d(3.821, 2.879, Rotation2d.fromDegrees(60)),
        new Pose2d(4.858, 2.711, Rotation2d.fromDegrees(120)),
        new Pose2d(5.144, 2.879, Rotation2d.fromDegrees(120)),
        new Pose2d(5.810, 3.689, Rotation2d.fromDegrees(180)),
        new Pose2d(5.810, 4.018, Rotation2d.fromDegrees(180)),
        new Pose2d(5.438, 5.007, Rotation2d.fromDegrees(240)),
        new Pose2d(5.154, 5.167, Rotation2d.fromDegrees(240))
    );

    public static final double FIELD_WIDTH_METERS = 17.56;
    public static final double FIELD_HEIGHT_METERS = 8.05;

    //both are in botspace. decreasing Y moves the bot back, decreasing X moves it right.
    public static final double X_OFFSET = -0.285;
    public static final double Y_OFFSET = 0.1;
}
