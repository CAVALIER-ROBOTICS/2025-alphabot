// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;

public class ElevatorSubsystem extends SubsystemBase {


  SparkMax primary = new SparkMax(ElevatorSubsystemConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  SparkMax secondary = new SparkMax(ElevatorSubsystemConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  SparkMax spinGrabber = new SparkMax(ElevatorSubsystemConstants.GRABBER_MOTOR_ID, MotorType.kBrushless);
  
  ColorSensorV3 coralSensor = new ColorSensorV3(Port.kOnboard);

  RelativeEncoder rightEncoder = primary.getEncoder();

  PIDController pid = new PIDController(0.01, 0, 0);


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    SparkMaxConfig smc = new SparkMaxConfig();
    smc.follow(primary, false);
    secondary.configure(smc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig neoConfig = new SparkMaxConfig();
    neoConfig.smartCurrentLimit(ElevatorSubsystemConstants.NEO550_CURRENT_LIMIT);
    spinGrabber.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig primaryConfig = new SparkMaxConfig();
    primaryConfig.encoder.positionConversionFactor(1.0);
    primary.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightEncoder.setPosition(0.0);
    pid.setTolerance(.2);
  }

  public void setSpin(double percent)
  {
    primary.set(percent);
  }

  public void setGrabber(double percent)
  {
    spinGrabber.set(percent);
  }

  public double getPosition()
  {
    return rightEncoder.getPosition();
  }

  public void setPosition(double position)
  {
    double speed = pid.calculate(getPosition(), position);
    setSpin(speed);
  }

  public void stopAll() {
    setSpin(0.0);
    setGrabber(0.0);    
  }

  public boolean getIsCoralInHoldingPosition() {
    return coralSensor.getProximity() > ElevatorSubsystemConstants.CORAL_SENSOR_PROXIMITY_THRESHOLD;
  }

  public boolean isElevatorPIDAtSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElevatorPos",getPosition());
  }
}
