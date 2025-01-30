// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.utils.NeoKrakenModule;

public class ElevatorSubsystem extends SubsystemBase {


  SparkMax spinRight = new SparkMax(ElevatorSubsystemConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  SparkMax spinLeft = new SparkMax(ElevatorSubsystemConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  SparkMax spinGrabber = new SparkMax(ElevatorSubsystemConstants.GRABBER_MOTOR_ID, MotorType.kBrushless);

  RelativeEncoder rightEncoder = spinRight.getEncoder();
  RelativeEncoder leftEncoder = spinLeft.getEncoder();

  PIDController pid = new PIDController(0, 0, 0);


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem()
  {}

  public void setSpinUp(double percent)
  {
    spinRight.set(percent);
    spinLeft.set(percent);
  }
  public void setSpinDown(double percent)
  {
    spinRight.set(-percent);
    spinLeft.set(-percent);
  }
  public void setGrabberIn(double percent)
  {
    spinGrabber.set(percent);
  }
  public void setGrabberOut(double percent)
  {
    spinGrabber.set(-percent);
  }
  public void setIdle()
  {
    spinRight.set(0);
    spinLeft.set(0);
  }
  public double getPosition()
  {
    return rightEncoder.getPosition();
  }
  public void setPosition(double position)
  {
    double speed = pid.calculate(getPosition(), position);
    setSpinUp(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElevatorPos",getPosition());
  }
}
