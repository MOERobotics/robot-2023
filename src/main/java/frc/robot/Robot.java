// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.autonomousgraveyard.*;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.TherMOEDynamic;
import frc.robot.generic.swerveBot;
import frc.robot.teleop.DriveCode;
import frc.robot.teleop.GenericTeleop;
import frc.robot.vision.MoeNetVision;
import edu.wpi.first.wpilibj.DriverStation;


public class Robot extends TimedRobot {
  Joystick autoSelect = new Joystick(0);
  public static final genericAutonomous
        A1C = new A1C(),
        A1B2C = new A1B2C(),
        A1 = new A1(),
        FEngage = new FEngage(),
        F2Engage = new F2Engage(),
        ScoreAndStop = new ScoreAndStop(),
        A1BHigh = new A1BHigh();

  genericAutonomous autonomous = new A1BHigh();
  GenericTeleop teleop = new DriveCode();
  DriverStation.Alliance OurAllianceColor;
  GenericRobot robot = new TherMOEDynamic();
  MoeNetVision vision = new MoeNetVision(robot);
  Field2d field = new Field2d();

  @Override
  public void robotInit() {
   robot.resetPigeon();
   robot.resetAttitude();

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
   SmartDashboard.putBoolean("isAuto", false);
  }


  @Override
  public void robotPeriodic() {

   SmartDash.Dashboard(robot);
   SmartDashboard.putString("autonomous", autonomous.getClass().getName());
   OurAllianceColor = DriverStation.getAlliance();

   if (OurAllianceColor == DriverStation.Alliance.Red) {
     robot.setRed(true);
   }
   else {
     robot.setRed(false);
   }
   SmartDashboard.putBoolean("isAuto", false);
   SmartDashboard.putNumber("TOF Distance", robot.getTOFDistance());
   if (Math.abs(robot.getPitch()) > 10 || Math.abs(robot.getRoll()) > 10){
       robot.robotTipping(true);
   }
   else{
       robot.robotTipping(false);
   }
  }


  @Override
  public void autonomousInit() {
    SmartDashboard.putBoolean("isAuto", true);
    autonomous.autonomousInit(robot);
  }


  @Override
  public void autonomousPeriodic() {
      SmartDashboard.putBoolean("isAuto", true);
    autonomous.autonomousPeriodic(robot);
  }


  @Override
  public void teleopInit() {
    SmartDashboard.putBoolean("isAuto", false);
    teleop.teleopInit(robot);
  }


  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("isAuto", false);
    teleop.teleopPeriodic(robot);
  }


  @Override
  public void disabledInit() {SmartDashboard.putBoolean("isAuto", false);}


  @Override
  public void disabledPeriodic() {
   SmartDashboard.putBoolean("isAuto", false);
   //if (autoSelect.getRawButtonPressed(1))
   if (autoSelect.getRawButtonPressed(2)) autonomous = A1C;
   //if (autoSelect.getRawButtonPressed(3))
   if (autoSelect.getRawButtonPressed(4)) autonomous = A1B2C;
   if (autoSelect.getRawButtonPressed(5)) autonomous = A1BHigh;
   if (autoSelect.getRawButtonPressed(6)) autonomous = A1;
   if (autoSelect.getRawButtonPressed(7)) autonomous = FEngage;
   if (autoSelect.getRawButtonPressed(8)) autonomous = F2Engage;
   if (autoSelect.getRawButtonPressed(9)) autonomous = ScoreAndStop;

  }


  @Override
  public void testInit() {
   SmartDashboard.putBoolean("isAuto", false);
   SmartDashboard.putData("Field", field);
   robot.setPose();
  }


  @Override
  public void testPeriodic() {
   SmartDashboard.putBoolean("isAuto", false);
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
