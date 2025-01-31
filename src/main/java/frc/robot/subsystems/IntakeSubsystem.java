// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  SparkMax intakeSpin = new SparkMax(IntakeSubsystemConstants.INTAKE_SPIN_MOTORID,MotorType.kBrushless);
  SparkMax intakeAngle = new SparkMax(IntakeSubsystemConstants.INTAKE_ANGLE_MOTORID,MotorType.kBrushless);
  PIDController pid = new PIDController(0,0,0);
  DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeSubsystemConstants.DUTY_CYCLE_ENCODER_ID);

  public IntakeSubsystem() 
  {
  }

  public double getAbsolutePosition()
  {
    return encoder.get();
  }

  public void setAnglePercent(double percent)
  {
    intakeAngle.set(percent);
  }

  public void setIntakeSpin(double point) 
  {
    intakeSpin.set(point);
  }

  public void setPosition(double position)
  {
    double setpoint = pid.calculate(getAbsolutePosition(), position);
    setAnglePercent(setpoint);
  }

  public void stopAll() {
    intakeAngle.set(0.0);
    intakeSpin.set(0.0);
  }

  public double getFlywheelCurrentDraw() {
    return intakeSpin.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putNumber("Intake Encoder" , encoder.get());
  }

}
