package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class autoBalance extends genericCommand {

    double base = 18.0;
    double angleOfBoard = .19;
    double basePower = 35.0;
    double climbPower = 25.0;
    double correctionPower = 13.0;

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
    double desiredPitch = 9.0;
    double firstBreak = 5;
    double high = 13.5;

    double dropping = 10;
    double totalPathLength = 0;
    double initpos;
    double boundPos1;
    double boundPos2;
    double boundPos3;
    boolean canClimb = false;
    double xspd, timerDelta;
    Timer m_timer = new Timer();
    @Override
    public void init(GenericRobot robot) {
        SmartDashboard.putString("Here i am", "hello im in init");
        autoStep = 0;
        canClimb = false;
        timerDelta = 0;
    }


    @Override
    public void periodic(GenericRobot robot) {
        SmartDashboard.putNumber("commandautonomousStep", autoStep);
        SmartDashboard.putString("Here i am", "hello im in periodic");
////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
        Pose2d robotPose = robot.getPose();
        currPitch = robot.getPitch(); //test switching roll and pitch
        currRoll = robot.getRoll();
        if (robot.getYaw() > 50 && robot.getYaw() < 130){
            currPitch = -robot.getRoll();
            canClimb = true;
        }
        if (robot.getYaw() < -50 && robot.getYaw() > -130){
            currPitch = robot.getRoll();
            canClimb = true;
        }

        if (!canClimb){
            autoStep = 5;
        }
        switch (autoStep) {
            case 0:
                xspd = basePower;
                if (Math.abs(currPitch) > firstBreak) {
                    //Add length of the robot from front encoder to end of back wheel.
                    boundPos1 = robotPose.getX();
                    boundPos2 = boundPos1 + 29 - (totalPathLength + 56);
                    boundPos3 = boundPos1 + 56 - (totalPathLength - 56);
                    autoStep++;
                }
                break;
            case 1:
                xspd = basePower;
                if (Math.abs(currPitch) > high) {
                    autoStep++;
                }
                break;
            case 2:
                xspd = climbPower;
                if (Math.abs(currPitch) < dropping) {
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }
                break;
            case 3:
                xspd = -correctionPower;
                //This is a future feature to stop and let others get on before autobalancing.
                autoStep++;
                break;
            case 4:
                if ((currPitch < -desiredPitch) && (m_timer.get() <1)) { //correcting begins
                    timerDelta = m_timer.get();
                    xspd = -correctionPower; //backward
                } else if ((currPitch > desiredPitch) && (m_timer.get() <1)) {
                    xspd = correctionPower;//forward
                    timerDelta=m_timer.get();
                } else {
                    xspd = 0;
                    if (m_timer.get()-timerDelta > .25) {
                        m_timer.reset();
                        m_timer.start();
                    }
                }
                break;
        }
        robot.setDrive(xspd, 0, 0);
    }
}