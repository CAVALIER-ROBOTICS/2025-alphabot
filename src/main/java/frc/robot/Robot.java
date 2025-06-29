// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.AutoAlignCommandFactory;
import frc.robot.utils.CavbotsPhotonCamera;
import frc.robot.utils.CavbotsPoseEstimator;
import frc.robot.utils.PathLoader;

@SuppressWarnings("unused")
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  Field2d field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();
    AutoAlignCommandFactory.initalize();
    m_autonomousCommand = PathLoader.loadAuto("autoTop");
  }

  @Override
  public void robotPeriodic() {
    field.setRobotPose(m_robotContainer.driveSubsystem.getPoseEstimator().getPose2d());
    SmartDashboard.putData("Field", field);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
    AutoAlignCommandFactory.initalize();
    Pose2d poseAt = new Pose2d(14.860, 4.205, new Rotation2d());
    Pose2d nearest = (AutoAlignCommandFactory.getClosestPose(poseAt, true, true));
    Field2d field = new Field2d();
    Field2d field2 = new Field2d();

    field2.setRobotPose(poseAt);
    field.setRobotPose(nearest);

    SmartDashboard.putData("alignToPose", field);
    SmartDashboard.putData("atPose", field2);
  }

  @Override
  public void simulationPeriodic() {}
}
