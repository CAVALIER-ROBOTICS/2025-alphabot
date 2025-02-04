package frc.robot.Constants;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
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
        new Pose2d(4.958, 2.748, Rotation2d.fromDegrees(123.311)),
        new Pose2d(5.244, 2.913, Rotation2d.fromDegrees(123.311)),
        new Pose2d(5.860, 3.755, Rotation2d.fromDegrees(180)),
        new Pose2d(5.860, 4.095, Rotation2d.fromDegrees(180)),
        new Pose2d(5.409, 5.092, Rotation2d.fromDegrees(-120.964)),
        new Pose2d(5.124, 5.242, Rotation2d.fromDegrees(-120.964)),
        new Pose2d(5.409, 5.092, Rotation2d.fromDegrees(-120.964)),
        new Pose2d(5.124, 5.242, Rotation2d.fromDegrees(-120.964)),
        new Pose2d(4.027, 5.347, Rotation2d.fromDegrees(-58.782)),
        new Pose2d(3.741, 5.182, Rotation2d.fromDegrees(-58.782)),
        new Pose2d(3.125, 4.295, Rotation2d.fromDegrees(0.0)),
        new Pose2d(3.095, 3.965, Rotation2d.fromDegrees(0.0)),
        new Pose2d(3.561, 2.958, Rotation2d.fromDegrees(59.534)),
        new Pose2d(3.877, 2.808, Rotation2d.fromDegrees(59.534))
    );

    public static final double FIELD_WIDTH_METERS = 17.588;

    public static final double MAXIMUM_DISTANCE_FROM_GOAL_METERS = .1;

    public static final PIDController X_PRECISE_PATH_PID = new PIDController(5.0, 0.0, 0.0);
    public static final PIDController Y_PRECISE_PATH_PID = new PIDController(5.0, 0.0, 0.0);
}
