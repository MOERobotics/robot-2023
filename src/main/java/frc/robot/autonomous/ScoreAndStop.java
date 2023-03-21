package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import frc.robot.vision.Detection;
import frc.robot.vision.MoeNetVision;
import org.opencv.core.Point;

public class ScoreAndStop extends genericAutonomous {
    Point startPosition       = new Point(85,108);
    Point firstScorePosition  = new Point(69, 108);
    Point startPositionBlue       = new Point(85,108);
    Point firstScorePositionBlue  = new Point(69, 108);
    Rotation2d startRot;
    Timer m_timer = new Timer();
    boolean lightOn, collectorUp, openGripper;
    double defaultPower = 30.0;
    double xspd, yspd, turnspd;
    double xPos;
    double armPos = 0;
    double collectorRPM = 9000;
    PIDController PID = new PIDController(1.0e-1, 0, 0);
    double startXPose;
    double lengthOfField = 650;
    double centerLineBlue = 295;
    double centerLine = centerLineBlue;

    @Override
    public void autonomousInit(GenericRobot robot) {
        xspd = yspd = turnspd = 0;
        autonomousStep = 0;
        robot.setPigeonYaw(0);
        defaultPower = 35;
        startRot = new Rotation2d(0);
        centerLine = centerLineBlue;
        startPosition.x        = startPositionBlue.x;
        firstScorePosition.x   = firstScorePositionBlue.x;
        if (robot.getRed()){
            centerLine = lengthOfField - centerLineBlue;
            startPosition.x        = lengthOfField - startPositionBlue.x;
            firstScorePosition.x   = lengthOfField - firstScorePositionBlue.x;
            robot.setPigeonYaw(180);
        }
        robot.resetPose();
        robot.resetAttitude();
        robot.setPose(new Pose2d(startPosition.x, startPosition.y, startRot));
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("autoStep", autonomousStep);
        Pose2d currPose = robot.getPose();
        switch(autonomousStep) {
            case 0:
                collectorRPM = 0;
                robot.resetPose();
                collectorUp = true;
                openGripper = false;
                armPos = 101.1;
                m_timer.restart();
                if (Math.abs(robot.getPotDegrees() - armPos) <= 10) {
                    startXPose = currPose.getX();
                    autonomousStep++;
                }
                break;
            case 1: //rollback to get ready to score
                xspd = -30;
                if (robot.getRed()) xspd *= -1;
                if (Math.abs(startXPose - currPose.getX()) >= 24) {
                    xspd = yspd = 0;
                    m_timer.restart();
                    autonomousStep++;
                }
                break;
            case 2: //score the cone
                openGripper = true;
                armPos = 85;
                if (m_timer.get() > .2) {
                    m_timer.restart();
                    autonomousStep++;
                    startXPose = currPose.getX();
                }
                break;
            case 3: //drive forward
                xspd = defaultPower;
                yspd = turnspd = 0;
                xPos = robot.getPose().getX();
                if (xPos > 220) {
                    xspd = 0;
                    autonomousStep++;
                }
                break;
            case 4:
                xspd = 0;
                break;

        }
        if (currPose.getX() < centerLine && robot.getRed()){
            xspd = 0;
        }
        if (currPose.getX() > centerLine && !robot.getRed()){
            xspd = 0;
        }
        robot.raiseTopRoller(collectorUp);
        robot.setDrive(xspd, yspd, turnspd, true, true);
        robot.collect(collectorRPM);
        robot.openGripper(openGripper);
        robot.setLightsOn(lightOn);
        robot.holdArmPosition(armPos);
    }
}
