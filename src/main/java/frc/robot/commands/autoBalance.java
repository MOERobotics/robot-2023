package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class autoBalance extends genericCommand {

    double base = 18.0;
    double angleOfBoard = .19;
    double basePower = 35.0;
    double climbPower = 36.0;
    double correctionPower = 30.0;

    boolean resetting = false;
    double currPitch;
    double currRoll;
    double curPosOnRamp;
    double currPosInAutoBalance;

    double leftside;
    double rightside;
    int autoStep;
    double currentpos;
    double initPos;
    double startingPose;
    double desiredPitch = 4.0;
    double firstBreak = 5;
    double high = 11;
    double totalPathLength = 0;
    double initpos;
    double boundPos1;
    double boundPos2;
    double boundPos3;

    @Override
    public void init(GenericRobot robot) {
        SmartDashboard.putString("Here i am", "hello im in init");
        autoStep = 0;
    }


    @Override
    public void periodic(GenericRobot robot) {
        SmartDashboard.putNumber("commandautonomousStep", autoStep);
        SmartDashboard.putString("Here i am", "hello im in periodic");
////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
        Pose2d robotPose = robot.getPose();

        currPitch = robot.getPitch(); //test switching roll and pitch
        currRoll = robot.getRoll();

        switch (autoStep) {
            case 0:
                robot.setDrive(basePower, 0, 0);
                if (Math.abs(currPitch) > firstBreak) {
                    //Add length of the robot from front encoder to end of back wheel.
                    boundPos1 = robotPose.getX();
                    boundPos2 = boundPos1 + 29 - (totalPathLength + 56);
                    boundPos3 = boundPos1 + 56 - (totalPathLength - 56);
                    autoStep++;
                }
                break;
            case 1:
                robot.setDrive(basePower, 0, 0);
                if (Math.abs(currPitch) > high) {
                    autoStep++;
                }
                break;
            case 2:
                robot.setDrive(climbPower, 0, 0);
                if (Math.abs(currPitch) < desiredPitch) {
                    autoStep++;
                }
                break;
            case 3:
                robot.setDrive(-correctionPower, 0, 0);
                //This is a future feature to stop and let others get on before autobalancing.
                autoStep++;
                break;
            case 4:
                currPosInAutoBalance = robotPose.getX();
                if (currPitch < -desiredPitch) {
                    if (currPosInAutoBalance > boundPos2) {
                        robot.setDrive(-correctionPower, 0, 0);
                        initPos = robotPose.getX();
                    } else {
                        robot.setDrive(0, 0, 0);
                        initPos = robotPose.getX();
                        autoStep++;
                    }
                } else if (currPitch > desiredPitch) {
                    if (currPosInAutoBalance < boundPos3) {
                        robot.setDrive(correctionPower, 0, 0);
                        initPos = robotPose.getX();
                    } else {
                        robot.setDrive(0, 0, 0);
                        initPos = robotPose.getX();
                        autoStep++;
                    }
                } else {
                    robot.setDrive(0, 0, 0);
                    initPos = robotPose.getX();
                    autoStep++;
                }
                break;
            case 5:
                if (Math.abs(currPitch) > desiredPitch) {
                    autoStep--;
                } else {
                    robot.setDrive(0, 0, 0);
                }
                break;
        }
    }
}