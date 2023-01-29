package frc.robot.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class DriveCode extends GenericTeleop{

    boolean resetting = false;
    Joystick swerveStick = new Joystick(1);

    double currPitch;
    double currRoll;
    double curPosOnRamp;
    double currPosInAutoBalance;

    double leftside;
    double rightside;
    int autoStep;
    double currentpos;
    double initPos;
    double desiredPitch = 6.0;
    double initpos;
    double boundPos1;
    double boundPos2;
    double boundPos3;

    double currentHead;
    double error;
    double correction;
    double desiredAngle;
    double startingPos;

    double startAngle;
    boolean btnLeft = false;
    boolean btnRight = false;
    boolean autoBalance;
    int count;
    double totalPathLength = 0;

    @Override
    public void teleopInit(GenericRobot robot) {
        resetting = false;
        robot.setOffsetLeftA();
        robot.setOffsetLeftB();
        robot.setOffsetRightA();
        robot.setOffsetRightB();

        robot.resetAttitude();
        robot.resetPIDPivot();

        robot.resetStartHeading();

        robot.resetStartDists();

        robot.resetStartPivots();
    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {


        if (resetting) robot.resetAttitude();
        if (!resetting || (resetting && Math.abs(robot.getYaw()) < 1)) {
            resetting = false;
////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
            Pose2d robotPose = robot.getPose();

////////////////////////////////////////////////////////////////////////////////////////////////////////////Swerve code
            double xspd = robot.deadzone(-swerveStick.getRawAxis(1), .35) * robot.getMaxInchesPerSecond() / 2;
            double yspd = robot.deadzone(-swerveStick.getRawAxis(0), .35) * robot.getMaxInchesPerSecond() / 2;
            double turnspd = robot.deadzone(-swerveStick.getRawAxis(4), .35) * robot.getMaxRadPerSec() / 2;

            currPitch = robot.getPitch(); //test switching roll and pitch
            currRoll = robot.getRoll();

            //TODO: Delete later, added to push code
            SmartDashboard.putNumber("poseX",robotPose.getX());
            SmartDashboard.putNumber("poseY",robotPose.getY());
            SmartDashboard.putNumber("poseZ",robotPose.getRotation().getDegrees());
            SmartDashboard.putNumber("bound1", boundPos1);
            SmartDashboard.putNumber("bound2", boundPos2);
            SmartDashboard.putNumber("bound3", boundPos3);
            SmartDashboard.putNumber("Pitch", robot.getPitch());
            SmartDashboard.putNumber("Roll", robot.getRoll());
            SmartDashboard.putNumber("Yaw", robot.getYaw());
            SmartDashboard.putNumber("xspd", xspd);
            SmartDashboard.putNumber("yspd", yspd);
            SmartDashboard.putNumber("turnspd", turnspd);



            double base = 18.0;
            double angleOfBoard = .19;
            double basePower = 4.0;
            double climbPower = 2.4;
            double correctionPower = 2.0;

            if (swerveStick.getRawButton(5)) { // for varun
                turnspd *= 2;
            }
            if (swerveStick.getRawButton(6)) {
                xspd *= 2;
                yspd *= 2;
            }

            curPosOnRamp = 0;

            if(swerveStick.getRawButton(7)) {
                robot.setDrive(0, 0,0);
                switch (autoStep) {
                    case 0:
                        robot.setDrive(basePower,0,0);
                        if (Math.abs(currPitch)> 5) {

                            boundPos1 = robotPose.getX();//Add length of the robot from front encoder to end of back wheel.
                            boundPos2 = boundPos1+30-(totalPathLength+57);
                            boundPos3 = boundPos1+57-(totalPathLength-57);
                            autoStep += 1;



                        }
                        break;
                    case 1:
//                        curPosOnRamp = base * Math.sin(currPitch) * (Math.cos(angleOfBoard) / Math.sin(angleOfBoard));
//                        leftside = basePower*(base - curPosOnRamp)/base;
//                        rightside = basePower*(base - curPosOnRamp)/base;
                        robot.setDrive(basePower,0,0);
                        if (Math.abs(currPitch) >11){
                            autoStep += 1;
                        }
                        break;
                    case 2:
                        robot.setDrive(climbPower,0,0);
                        if(Math.abs(currPitch) < 10){
                            autoStep++;
                        }
                        break;
                    case 3:
                        //initPos = robotPose.getX() ;
                        robot.setDrive(-correctionPower,0,0);
                        //This is a future feature to stop and let others get on before autobalancing.
                    /*if(Math.abs(curPitch)<15){
                        leftside = 0;
                        rightside = 0;
                    }
                    if(leftJoystick.getRawButtonPressed(9)) {
                        autoStep++;
                    }*/
                        autoStep++;
                        break;
                /*case 4:
                    currentpos = robotPose.getY() ;
                    leftside = -climbPower;
                    rightside = -climbPower;
                    if(Math.abs(initPos - currentpos) > 1){
                        leftside = 0;
                        rightside = 0;
                        autoStep++;
                    }
                    break;
                case 5:
                    leftside = 0;
                    rightside = 0;
                    if(Math.abs(currPitch) < 2){
                        autoStep++;
                    }
                    break;*/
                    case 4:
                        currPosInAutoBalance = robotPose.getX() ;
                        if(currPitch<-desiredPitch){
                            if(currPosInAutoBalance > boundPos2){
                                robot.setDrive(-correctionPower,0,0);
                                initPos = robotPose.getX();
                            }else{
                                robot.setDrive(0, 0,0);
                                initPos = robotPose.getX() ;
                                autoStep++;
                            }
                        } else if(currPitch>desiredPitch){
                            if(currPosInAutoBalance < boundPos3){
                                robot.setDrive(-correctionPower,0,0);
                                initPos = robotPose.getX() ;
                            } else{
                                robot.setDrive(0, 0,0);
                                initPos = robotPose.getX() ;
                                autoStep++;
                            }
                        } else{
                            robot.setDrive(0, 0,0);
                            initPos = robotPose.getX() ;
                            autoStep++;
                        }
                        break;
                    case 5:
                        if(Math.abs(currPitch) > desiredPitch){
                            autoStep--;
                        } else{
                            robot.setDrive(0, 0,0);
                        }
                        break;
                }
            } else{
                robot.setDrive(xspd, yspd, turnspd);
                autoStep = 0;
            }



            if (swerveStick.getRawButton(1)) {
                resetting = true;
                robot.setOffsetLeftA();
                robot.setOffsetLeftB();
                robot.setOffsetRightA();
                robot.setOffsetRightB();

                robot.resetAttitude();
                robot.resetPIDPivot();

                robot.resetStartHeading();

                robot.resetStartDists();

                robot.resetStartPivots();
            }
        }
        SmartDashboard.putBoolean("I am resetting", resetting);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////end swerve code

/////////////////////////////////////////////////////////////////////////////////////////////////////////////start auto balance
        double current_roll = robot.getRoll();
        double current_pitch = robot.getPitch();

        //TODO: find actual inches of robot
        double base = 18.0;
        //TODO: see if i can just delete angle of board
        double angle_of_board = 0.19;

        double power_percentage = 0.5;
        double climb_percentage = 0.4;
        double correction_percentage = 0.25;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////end auto balance


    }


}
