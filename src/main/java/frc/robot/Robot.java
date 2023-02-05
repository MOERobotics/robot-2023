// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.swerveBot;
import frc.robot.teleop.DriveCode;
import frc.robot.teleop.GenericTeleop;
import frc.robot.vision.MoeNetVision;


public class Robot extends TimedRobot {
  public static final GenericTeleop
          driveCode = new DriveCode();

 // GenericRobot robot = new SwerveBot();
  genericAutonomous autonomous = new baseAuto();
  GenericTeleop teleop = driveCode;
  GenericRobot robot = new swerveBot();

  MoeNetVision vision = new MoeNetVision(robot);
  Field2d field = new Field2d();

  @Override
  public void robotInit() {
   robot.resetPigeon();
  }


  @Override
  public void robotPeriodic() {
   SmartDashboard.putNumber("yaw", robot.getYaw());
   SmartDashboard.putNumber("leftApivot", robot.getPivotLeftMotorA());
   SmartDashboard.putNumber("leftBpivot", robot.getPivotLeftMotorB());
   SmartDashboard.putNumber("rightApivot", robot.getPivotRightMotorA());
   SmartDashboard.putNumber("rightBpivot", robot.getPivotRightMotorB());

   SmartDashboard.putNumber("leftApivotRaw", robot.rawEncoderLeftA());
   SmartDashboard.putNumber("leftBpivotRaw", robot.rawEncoderLeftB());
   SmartDashboard.putNumber("rightApivotRaw", robot.rawEncoderRightA());
   SmartDashboard.putNumber("rightBpivotRaw", robot.rawEncoderRightB());
   SmartDashboard.putNumber("pitch", robot.getPitch());
   SmartDashboard.putNumber("roll", robot.getRoll());
   SmartDashboard.putNumber("pigeonYaw", robot.getPigeonYaw());
   SmartDashboard.putNumber("pigeonPitch", robot.getPigeonPitch());
   SmartDashboard.putNumber("pigeonRoll", robot.getPigeonRoll());
   SmartDashboard.putNumber("pigeonCompass", robot.getAbsoluteCompassHeadingPigeon());

   robot.getDriveDistanceInchesLeftA();
   robot.getDriveDistanceInchesLeftB();
   robot.getDriveDistanceInchesRightB();
   robot.getDriveDistanceInchesRightA();
  }


  @Override
  public void autonomousInit() {
    autonomous.autonomousInit(robot);
  }


  @Override
  public void autonomousPeriodic() {
    autonomous.autonomousPeriodic(robot);
    vision.genericPeriodic();
  }


  @Override
  public void teleopInit() {
    driveCode.teleopInit(robot);
  }


  @Override
  public void teleopPeriodic() {
    driveCode.teleopPeriodic(robot);
   vision.genericPeriodic();
  }


  @Override
  public void disabledInit() {}


  @Override
  public void disabledPeriodic() {
   vision.disabledPeriodic();
  }


  @Override
  public void testInit() {
   SmartDashboard.putData("Field", field);

  }


  @Override
  public void testPeriodic() {
   var pose = vision.getPose();
   if (vision.poseFound()){
      field.setRobotPose(pose.toPose2d());
   }
   SmartDashboard.putData("Field", field);
   SmartDashboard.putNumber("field x", field.getRobotPose().getX());
   SmartDashboard.putNumber("field y", field.getRobotPose().getY());
  }


  @Override
  public void simulationInit() {}


  @Override
  public void simulationPeriodic() {}
}
