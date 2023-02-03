package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.generic.GenericRobot;

public class autoBalanceBackward extends genericCommand {


    Joystick swerveStick = new Joystick(1);

    boolean resetting = false;
    double currPitch;
    double currRoll;
    double curPosOnRamp;
    double currPosInAutoBalance;

    double leftside;
    double rightside;
    int autoStepback;
    int autoSequenceStep;
    double currentpos;
    double initPos;
    double startingPose;
    double desiredPitch = 9.0;
    double totalPathLength = 0;
    double initpos;
    double boundPos1;
    double boundPos2;
    double boundPos3;

    @Override
    public void init(GenericRobot robot) {

    }

    double base = 18.0;
    double angleOfBoard = .19;
    double basePower = -35.0;
    double climbPower = -30.0;
    double correctionPower = -14.0;

    @Override
    public void periodic(GenericRobot robot) {

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


            switch (autoStepback) {
                case 0:
                    robot.setDrive(basePower, 0, 0);
                    if (Math.abs(currPitch) > 5) {
                        autoStepback += 1;


                    }
                    break;
                case 1:
//                        curPosOnRamp = base * Math.sin(currPitch) * (Math.cos(angleOfBoard) / Math.sin(angleOfBoard));
//                        leftside = basePower*(base - curPosOnRamp)/base;
//                        rightside = basePower*(base - curPosOnRamp)/base;
                    robot.setDrive(basePower, 0, 0);
                    if (Math.abs(currPitch) > 11) {
                        autoStepback += 1;
                    }
                    break;
                case 2:
                    robot.setDrive(climbPower, 0, 0);
                    if (Math.abs(currPitch) < 10) {
                        autoStepback++;
                    }
                    break;
                case 3:
                    //initPos = robotPose.getX() ;
                    robot.setDrive(-correctionPower, 0, 0);
                    //This is a future feature to stop and let others get on before autobalancing.
                    /*if(Math.abs(curPitch)<15){
                        leftside = 0;
                        rightside = 0;
                    }
                    if(leftJoystick.getRawButtonPressed(9)) {
                        autoStep++;
                    }*/
                    autoStepback++;
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
                    currPosInAutoBalance = robotPose.getX();
                    if (currPitch < -desiredPitch) {
                        robot.setDrive(-correctionPower, 0, 0);

                    } else if (currPitch > desiredPitch) {
                        robot.setDrive(correctionPower, 0, 0);
                    } else {
                        robot.setDrive(0, 0, 0);
                        autoStepback++;
                    }
                    break;
                case 5:
                    if (Math.abs(currPitch) > desiredPitch) {
                        autoStepback--;
                    } else {
                        robot.setDrive(0, 0, 0);
                    }
                    break;
            }
        }
    }
}