package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import frc.robot.vision.MoeNetVision;
import org.opencv.core.Point;

public class E2Engage extends genericAutonomous{

    Point startPosition       = new Point(85,108);
    Point firstScorePosition  = new Point(69, 108);
    Point estimatedCubeSpot   = new Point(270, 130.5);
    Point spotBeforeEngage    = new Point(220, 108);
    Point startPositionBlue       = new Point(85,108);
    Point firstScorePositionBlue  = new Point(69, 108);
    Point estimatedCubeSpotBlue   = new Point(270, 130.5);
    Point spotBeforeEngageBlue    = new Point(220, 108);

    double dist1 = AutoCodeLines.getDistance(startPositionBlue, firstScorePositionBlue);
    double dist2 = AutoCodeLines.getDistance(estimatedCubeSpotBlue, spotBeforeEngageBlue);
    Pose2d desiredCubePos;
    double lengthOfField = 650;

    MoeNetVision vision;

    Rotation2d startRot;
    boolean lightOn, collectorUp, openGripper;
    Timer m_timer = new Timer();
    double armPos = 0;
    double xspd, yspd, turnspd, s, t, basePower, currPitch, climbPower, desiredPitch, dropping,high,xPos, firstBreak;
    double desiredInchesPerSecond = 60;
    double xPidK = 7;
    double yPidK = 7;

    @Override
    public void autonomousInit(GenericRobot robot) {
        xspd = yspd = turnspd = 0;
        vision = new MoeNetVision(robot);
        startRot = new Rotation2d(0);
        autonomousStep = 0;
        lightOn = false;
        robot.setPigeonYaw(0);
        startPosition.x        = startPositionBlue.x;
        firstScorePosition.x   = firstScorePositionBlue.x;
        estimatedCubeSpot.x    = estimatedCubeSpotBlue.x;
        spotBeforeEngage.x     = spotBeforeEngageBlue.x;
        if (robot.getRed()){
            startPosition.x        = lengthOfField - startPositionBlue.x;
            firstScorePosition.x   = lengthOfField - firstScorePositionBlue.x;
            estimatedCubeSpot.x    = lengthOfField - estimatedCubeSpotBlue.x;
            spotBeforeEngage.x     = lengthOfField - spotBeforeEngageBlue.x;
            startRot = new Rotation2d(Math.PI);
            robot.setPigeonYaw(180);
        }
        desiredCubePos = new Pose2d(estimatedCubeSpot.x, estimatedCubeSpot.y, startRot);
        robot.setPose(new Pose2d(startPosition.x, startPosition.y, startRot));
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        Pose2d currPose = robot.getPose();
        switch(autonomousStep){
            case 0:
                robot.resetPose();
                collectorUp = true;
                openGripper = false;
                armPos = 101.1;
                m_timer.restart();
                autonomousStep ++;
                break;
            case 1: //rollback to get ready to score
                t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s, t) + xPidK*(positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK*(positionFunctionY(s) - currPose.getY());
                if (s >= dist1){
                    xspd = yspd = 0;
                    m_timer.restart();
                    autonomousStep++;
                }
                break;
            case 2: //score the cone
                openGripper = false;
                if (m_timer.get() > .25){
                    armPos = -4;
                    m_timer.restart();
                    autonomousStep++;
                }
                break;
            case 3: //drive over the charge station
                xspd = basePower;
                yspd = turnspd = 0;
                if (Math.abs(currPitch) > firstBreak) {

                    //Add length of the robot from front encoder to end of back wheel.
                    autonomousStep++;
                }
                break;
            case 4:
                xspd = basePower;
                if (currPitch > high) {
                    xPos = robot.getPose().getX();
                    autonomousStep++;
                }
                break;
            case 5:
                xspd = climbPower;
                if (Math.abs(currPitch) < dropping) {
                    autonomousStep++;
                }
                break;
            case 6:
                xspd = climbPower+4;
                if (Math.abs(currPitch) > high){
                    autonomousStep ++;
                }
                break;
            case 7:
                xspd = climbPower+4;
                if (Math.abs(currPitch) < desiredPitch){
                    xPos = robot.getPose().getX();
                    autonomousStep ++;
                }
                break;
        }
    }

    @Override
    public double positionFunctionX(double s) {
        return super.positionFunctionX(s);
    }

    @Override
    public double positionFunctionY(double s) {
        return super.positionFunctionY(s);
    }

    @Override
    public double positionFunctionTheta(double s) {
        return super.positionFunctionTheta(s);
    }

    @Override
    public double velocityFunctionX(double s, double time) {
        return super.velocityFunctionX(s, time);
    }

    @Override
    public double velocityFunctionY(double s, double time) {
        return super.velocityFunctionY(s, time);
    }

    @Override
    public double velocityFunctionTheta(double s, double time) {
        return super.velocityFunctionTheta(s, time);
    }

    @Override
    public double getS(double time) {
        return super.getS(time);
    }

    @Override
    public double getdS(double time) {
        return super.getdS(time);
    }
}
