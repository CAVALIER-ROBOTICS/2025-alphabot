// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.RobotConstants;

public class ClimbSubsystem extends SubsystemBase {
  SparkMax primary = new SparkMax(ClimbConstants.PRIMARY_ID, MotorType.kBrushless);
  SparkMax follower = new SparkMax(ClimbConstants.FOLLOWER_ID, MotorType.kBrushless);
  RelativeEncoder enc = primary.getEncoder();

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    SparkMaxConfig primaryConfig = new SparkMaxConfig();
    primaryConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);
    primary.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig secondaryConfig = new SparkMaxConfig();
    secondaryConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);
    secondaryConfig.follow(primary, true);
    follower.configure(secondaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    enc.setPosition(0.0); //zero climb angle motor on startup.
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
