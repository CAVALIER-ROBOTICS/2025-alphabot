// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class ElevatorSubsystemConstants 
{
    public static final int LEFT_MOTOR_ID = 23;
    public static final int RIGHT_MOTOR_ID = 24;
    public static final int GRABBER_MOTOR_ID = 25;

    public static final int CORAL_SENSOR_PROXIMITY_THRESHOLD = 500;

    public static final int NEO550_CURRENT_LIMIT = 30;

    public static final double L1_ENCODER_POSITION = 34;
    public static final double L2_ENCODER_POSITION = 13.0;
    public static final double L3_ENCODER_POSITION = 34.25;
    public static final double HP_ENCODER_POSITION = 1.5;
    public static final double DEFAULT_POSITION = 1.5; //So the carriage doesn't slam into the base.

    public static final double GRABBER_SPEED = 0.5;
    public static final double L1_GRABBER_SPEED = .15;
    public static final double INTAKE_GRABBER_SPEED = 0.4;

    public static final double HOMED_CURRENT_DRAW = 75.0;

    public static final double MAX_ACCELERATION = 10000;
    public static final double MAX_VELOCITY = 10000;
    public static final double AT_SETPOINT_TOLERANCE = 1.0;

}
