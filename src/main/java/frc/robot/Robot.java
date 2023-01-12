// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.swerveBot;
import frc.robot.teleop.DriveCode;
import frc.robot.teleop.GenericTeleop;


public class Robot extends TimedRobot {
  public static final GenericTeleop
          driveCode = new DriveCode();

 // GenericRobot robot = new SwerveBot();

  GenericTeleop teleop = driveCode;
  GenericRobot robot = new swerveBot();


  @Override
  public void robotInit() {

  }


  @Override
  public void robotPeriodic() {}


  @Override
  public void autonomousInit() {}


  @Override
  public void autonomousPeriodic() {}


  @Override
  public void teleopInit() {
    driveCode.teleopInit(robot);
  }


  @Override
  public void teleopPeriodic() {
    driveCode.teleopPeriodic(robot);
  }


  @Override
  public void disabledInit() {}


  @Override
  public void disabledPeriodic() {}


  @Override
  public void testInit() {}


  @Override
  public void testPeriodic() {}


  @Override
  public void simulationInit() {}


  @Override
  public void simulationPeriodic() {}
}
