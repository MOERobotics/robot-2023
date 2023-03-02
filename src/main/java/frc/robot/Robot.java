// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
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
        A1CDock = new A1CDock(),
        A1C = new A1C(),
        ExitAndEngage = new ExitAndEngage();

  genericAutonomous autonomous = new A1CDock();
  GenericTeleop teleop = new DriveCode();
  DriverStation.Alliance OurAllianceColor;
  GenericRobot robot = new TherMOEDynamic();
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

   SmartDash.Dashboard(robot);
   SmartDashboard.putString("autonomous", autonomous.getClass().getName());
   OurAllianceColor = DriverStation.getAlliance();

   if (OurAllianceColor == DriverStation.Alliance.Red) {
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
   if (autoSelect.getRawButtonPressed(1)) autonomous = A1CDock;
   if (autoSelect.getRawButtonPressed(2)) autonomous = A1C;
   if (autoSelect.getRawButtonPressed(3)) autonomous = ExitAndEngage;
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
