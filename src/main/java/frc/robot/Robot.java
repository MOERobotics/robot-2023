// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.ExitAndEngage;
import frc.robot.autonomous.genericAutonomous;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.swerveBot;
import frc.robot.teleop.DriveCode;
import frc.robot.teleop.GenericTeleop;
import frc.robot.autonomous.VisionScoring;
import frc.robot.vision.MoeNetVision;
import edu.wpi.first.wpilibj.DriverStation;


public class Robot extends TimedRobot {

  genericAutonomous autonomous = new ExitAndEngage();
  GenericTeleop teleop = new DriveCode();
  DriverStation.Alliance OurAllianceColor;
  GenericRobot robot = new swerveBot();

  MoeNetVision vision = new MoeNetVision(robot);
  Field2d field = new Field2d();

  @Override
  public void robotInit() {
   robot.resetPigeon();

   OurAllianceColor = DriverStation.getAlliance();

   if (OurAllianceColor == DriverStation.Alliance.Red)
   {
    robot.setRed(true);
   }
   else {
    robot.setRed(false);
   }
   if (robot.getRed()){
       robot.setPigeonYaw(180);
   }
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

  SmartDashboard.putBoolean("Red Robot", robot.getRed());

   SmartDashboard.putNumber("armPosition", robot.getArmPosition());
   SmartDashboard.putBoolean("cargoInCollect", robot.cargoInCollector());


  robot.getDriveDistanceInchesLeftA();
  robot.getDriveDistanceInchesLeftB();
  robot.getDriveDistanceInchesRightB();
  robot.getDriveDistanceInchesRightA();

  OurAllianceColor = DriverStation.getAlliance();

  if (OurAllianceColor == DriverStation.Alliance.Red)
  {
   robot.setRed(true);
  }
  else {
   robot.setRed(false);
  }

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
    teleop.teleopInit(robot);
  }


  @Override
  public void teleopPeriodic() {
    teleop.teleopPeriodic(robot);
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
   robot.setPose();
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

   vision.genericPeriodic();
  }


  @Override
  public void simulationInit() {}


  @Override
  public void simulationPeriodic() {}
}
