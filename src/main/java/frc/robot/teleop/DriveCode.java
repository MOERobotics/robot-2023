package frc.robot.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autoBalance;
import frc.robot.commands.autoBalanceBackward;
import frc.robot.commands.genericCommand;
import frc.robot.generic.GenericRobot;

public class DriveCode extends GenericTeleop {

    boolean resetting = false;
    Joystick swerveStick = new Joystick(1);

    double currPitch;
    double currRoll;
    double curPosOnRamp;
    double currPosInAutoBalance;

    double leftside;
    double rightside;
    int autoStep;
    int autoSequenceStep;
    double currentpos;
    double initPos;
    double autoStepback;
    double startingPose;
    double desiredPitch = 9.0;
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
    boolean initialize = false;
    boolean initialize1 = false;

    genericCommand balance = new autoBalance();
    genericCommand balanceBack = new autoBalanceBackward();

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
            SmartDashboard.putNumber("poseX", robotPose.getX());
            SmartDashboard.putNumber("poseY", robotPose.getY());
            SmartDashboard.putNumber("poseZ", robotPose.getRotation().getDegrees());
            SmartDashboard.putNumber("bound1", boundPos1);
            SmartDashboard.putNumber("bound2", boundPos2);
            SmartDashboard.putNumber("bound3", boundPos3);
            SmartDashboard.putNumber("Pitch", robot.getPitch());
            SmartDashboard.putNumber("Roll", robot.getRoll());
            SmartDashboard.putNumber("Yaw", robot.getYaw());
            SmartDashboard.putNumber("xspd", xspd);
            SmartDashboard.putNumber("yspd", yspd);
            SmartDashboard.putNumber("turnspd", turnspd);
            SmartDashboard.putNumber("autostep", autoStep);
            SmartDashboard.putNumber("boundPos1", boundPos1);
            SmartDashboard.putNumber("boundPos2", boundPos2);
            SmartDashboard.putNumber("boundPos3", boundPos3);
            SmartDashboard.putNumber("autoStepback", autoStepback);


            double base = 18.0;
            double angleOfBoard = .19;
            double basePower = 35.0;
            double climbPower = 30.0;
            double correctionPower = 14.0;

            if (swerveStick.getRawButton(5)) { // for varun
                turnspd *= 2;
            }
            if (swerveStick.getRawButton(6)) {
                xspd *= 2;
                yspd *= 2;
                SmartDashboard.putNumber("xspd", xspd);
            }

            curPosOnRamp = 0;
            if (swerveStick.getRawButton(3)) {


                switch (autoSequenceStep) {
                    case 0:
                        startingPose = robotPose.getX();
                        robot.setDrive(basePower, 0, 0);
                        autoSequenceStep++;

                        break;
                    case 1:
                        if (robotPose.getX() >= startingPose + 12) {
                            robot.setDrive(0, 0, 0);
                            autoSequenceStep++;
                        }
                        break;
                    case 2:
                        startingPose = robotPose.getY();
                        robot.setDrive(0, basePower, 0);
                        autoSequenceStep++;
                        break;
                    case 3:
                        if (robotPose.getY() <= startingPose - 12) {
                            robot.setDrive(0, 0, 0);
                            autoSequenceStep++;
                        }
                        break;
                    case 4:
                        startingPose = robotPose.getX();
                        robot.setDrive(-basePower, 0, 0);
                        autoSequenceStep++;
                        break;
                    case 5:
                        if (robotPose.getX() <= startingPose - 12) {
                            robot.setDrive(0, 0, 0);
                            autoSequenceStep++;
                        }
                        break;
                    case 6:
                        startingPose = robotPose.getRotation().getDegrees();
                        robot.setDrive(0, 0, 2.5);
                        autoSequenceStep++;
                        break;
                    case 7:
                        if (robotPose.getRotation().getDegrees() <= startingPose - 90) {
                            robot.setDrive(0, 0, 0);
                            autoSequenceStep++;
                        }
                        break;
                    case 8:
                        startingPose = robotPose.getY();
                        robot.setDrive(0, basePower, 0);
                        autoSequenceStep++;
                        break;
                    case 9:
                        if (robotPose.getY() <= startingPose + 12) {
                            robot.setDrive(0, 0, 0);
                            autoSequenceStep++;
                        }
                        break;
                    case 10:
                        startingPose = robotPose.getRotation().getDegrees();
                        robot.setDrive(0, 0, -2.5);
                        autoSequenceStep++;
                        break;
                    case 11:
                        if (robotPose.getRotation().getDegrees() <= 90 && robotPose.getRotation().getDegrees() > 0) {
                            robot.setDrive(0, 0, 0);
                            autoSequenceStep++;
                        }
                        break;
                }
            } else {
                autoSequenceStep = 0;
            }

            if (swerveStick.getRawButton(7)) {
                balance.periodic(robot);
            } else {
                initialize = false;
                autoStep = 0;
            }
            if (swerveStick.getRawButton(8)) {
                balanceBack.periodic(robot);
            } else {
                initialize1 = false;
                autoStepback = 0;
            }
            if(!swerveStick.getRawButton(7)&&!swerveStick.getRawButton(8)){
                robot.setDrive(xspd,yspd,turnspd);
            }
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
