package frc.robot.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class DriveCode extends GenericTeleop{
    double[] startDists;
    double[] startPivots;
    double startHeading;
    double oldLeftA = 0;
    double oldLeftB = 0;
    double oldRightA = 0;
    double oldRightB = 0;

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

        startHeading = robot.getYaw();

        startDists = new double[] {robot.getDriveDistanceInchesLeftA(), robot.getDriveDistanceInchesRightA(),
                robot.getDriveDistanceInchesLeftB(),robot.getDriveDistanceInchesRightB()};

        startPivots = new double[] {robot.getPivotLeftMotorA(), robot.getPivotRightMotorA(),
                robot.getPivotLeftMotorB(), robot.getPivotRightMotorB()};
    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {


        if (resetting) robot.resetAttitude();
        if (!resetting || (resetting && Math.abs(robot.getYaw()) < 1)) {
            resetting = false;
////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
            Pose2d robotPose = robot.getPose(startHeading, robot.getYaw(), startDists, startPivots, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

////////////////////////////////////////////////////////////////////////////////////////////////////////////Swerve code
            double xspd = robot.deadzone(-swerveStick.getRawAxis(1), .35) * robot.getMaxMeterPerSec() / 2;
            double yspd = robot.deadzone(-swerveStick.getRawAxis(0), .35) * robot.getMaxMeterPerSec() / 2;
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
                driving(robot, 0, 0,0);
                switch (autoStep) {
                    case 0:
                        driving(robot, basePower,0,0);
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
                        driving(robot, basePower,0,0);
                        if (Math.abs(currPitch) >11){
                            autoStep += 1;
                        }
                        break;
                    case 2:
                        driving(robot, climbPower,0,0);
                        if(Math.abs(currPitch) < 10){
                            autoStep++;
                        }
                        break;
                    case 3:
                        //initPos = robotPose.getX() ;
                        driving(robot, -correctionPower,0,0);
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
                                driving(robot, -correctionPower,0,0);
                                initPos = robotPose.getX();
                            }else{
                                driving(robot, 0, 0,0);
                                initPos = robotPose.getX() ;
                                autoStep++;
                            }
                        } else if(currPitch>desiredPitch){
                            if(currPosInAutoBalance < boundPos3){
                                driving(robot, -correctionPower,0,0);
                                initPos = robotPose.getX() ;
                            } else{
                                driving(robot, 0, 0,0);
                                initPos = robotPose.getX() ;
                                autoStep++;
                            }
                        } else{
                            driving(robot, 0, 0,0);
                            initPos = robotPose.getX() ;
                            autoStep++;
                        }
                        break;
                    case 5:
                        if(Math.abs(currPitch) > desiredPitch){
                            autoStep--;
                        } else{
                            driving(robot, 0, 0,0);
                        }
                        break;
                }
            } else{
                driving(robot, xspd, yspd, turnspd);
            }



            if (swerveStick.getRawButton(1)) {
                resetting = true;
                robot.setOffsetLeftA();
                robot.setOffsetLeftB();
                robot.setOffsetRightA();
                robot.setOffsetRightB();

                robot.resetAttitude();
                robot.resetPIDPivot();

                startHeading = robot.getYaw();

                startDists = new double[]{robot.getDriveDistanceInchesLeftA(), robot.getDriveDistanceInchesRightA(),
                        robot.getDriveDistanceInchesLeftB(), robot.getDriveDistanceInchesRightB()};

                startPivots = new double[]{robot.getPivotLeftMotorA(), robot.getPivotRightMotorA(),
                        robot.getPivotLeftMotorB(), robot.getPivotRightMotorB()};
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

    public void driving(GenericRobot robot, double xspd, double yspd, double turnspd) {
        //everything from line 65-86 go to separate function, that's going to be drive function
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xspd,
                yspd, turnspd, Rotation2d.fromDegrees(-robot.getYaw()));
        SwerveModuleState[] moduleStates = robot.kinematics().toSwerveModuleStates(chassisSpeeds);
        SwerveModuleState frontLeftState = moduleStates[0],
                frontRightState = moduleStates[1],
                backLeftState = moduleStates[2],
                backRightState = moduleStates[3];

        frontLeftState = robot.optimizeSwervePivots(frontLeftState, Rotation2d.fromDegrees(robot.getPivotLeftMotorA()));
        frontRightState = robot.optimizeSwervePivots(frontRightState, Rotation2d.fromDegrees(robot.getPivotRightMotorA()));
        backLeftState = robot.optimizeSwervePivots(backLeftState, Rotation2d.fromDegrees(robot.getPivotLeftMotorB()));
        backRightState = robot.optimizeSwervePivots(backRightState, Rotation2d.fromDegrees(robot.getPivotRightMotorB()));

        if (xspd == 0 && yspd == 0 && turnspd == 0) {
            robot.stopSwerve(oldLeftA, oldRightA, oldLeftB, oldRightB);
        } else {
            robot.swerve(frontLeftState, frontRightState, backLeftState, backRightState);
            oldLeftA = frontLeftState.angle.getDegrees();
            oldLeftB = backLeftState.angle.getDegrees();
            oldRightA = frontRightState.angle.getDegrees();
            oldRightB = backRightState.angle.getDegrees();
        }

    }


}
