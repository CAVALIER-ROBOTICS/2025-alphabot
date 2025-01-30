package frc.robot.Constants;

import com.pathplanner.lib.path.PathConstraints;

public class PathingConstants {
    //find actual values
    public static final PathConstraints PATHFINDING_CONSTRAINTS = new PathConstraints(
        3.0, 
        4.0, 
        Math.toRadians(540), 
        Math.toRadians(270)
    ); 
}
