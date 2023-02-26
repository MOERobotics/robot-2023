package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import org.opencv.core.Point;

public class exampleAutonomous extends genericAutonomous{

    Point startPosition = new Point(35, 20);
    Point secondPosition = new Point(47, 70);
    Point thirdPosition = new Point(70, 47);
    Point endPosition = new Point(32, 18);

    double distanceOne = AutoCodeLines.getDistance(startPosition,secondPosition);
    double distanceTwo = AutoCodeLines.getDistance(secondPosition, thirdPosition);
    double distanceThree = AutoCodeLines.getDistance(thirdPosition, endPosition);

    double desiredInPerSec = 40;
    double ds = desiredInPerSec;
    double xspd,yspd,turnspd, s;

    double xPidK = 1.0e-3;
    double yPidK = 1.0e-3;
    private final Timer m_timer = new Timer();

    @Override
    public void autonomousInit(GenericRobot robot) {
        m_timer.reset();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetStartHeading();
        robot.setPose(new Pose2d(startPosition.x, startPosition.y, new Rotation2d(0)));
        autonomousStep = 0;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        Pose2d currPose = robot.getPose();
        switch(autonomousStep){
            case 0:
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = s = 0;
                autonomousStep ++;
                break;
            case 1:
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, m_timer.get()) + xPidK*(positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, m_timer.get()) + yPidK*(positionFunctionY(s) - currPose.getY());
                turnspd = velocityFunctionTheta(s, m_timer.get());
                if (s >= distanceOne){
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
            case 2:
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, m_timer.get()) + xPidK*(positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, m_timer.get()) + yPidK*(positionFunctionY(s) - currPose.getY());
                turnspd = velocityFunctionTheta(s, m_timer.get());
                if (s >= distanceTwo){
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
            case 3:
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, m_timer.get()) + xPidK*(positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, m_timer.get()) + yPidK*(positionFunctionY(s) - currPose.getY());
                turnspd = velocityFunctionTheta(s, m_timer.get());
                if (s >= distanceThree){
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
            case 4:
                m_timer.reset();
                xspd = yspd = turnspd = 0;
                break;
        }
        robot.setDrive(xspd,yspd,turnspd);
    }

    @Override
    public double positionFunctionX(double s) {
        if (autonomousStep == 0){
            return startPosition.x;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getPositionX(startPosition, secondPosition, s);
        }
        if (autonomousStep == 2){
            return AutoCodeLines.getPositionX(secondPosition, thirdPosition, s);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getPositionX(thirdPosition, endPosition, s);
        }
        return endPosition.x;
    }

    @Override
    public double positionFunctionY(double s) {
        if (autonomousStep == 0){
            return startPosition.y;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getPositionY(startPosition, secondPosition, s);
        }
        if (autonomousStep == 2){
            return AutoCodeLines.getPositionY(secondPosition, thirdPosition, s);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getPositionY(thirdPosition, endPosition, s);
        }
        return endPosition.y;
    }

    @Override
    public double positionFunctionTheta(double s) {
        return 0;
    }

    @Override
    public double velocityFunctionX(double s, double time) {
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityX(startPosition, secondPosition, s);
        }
        if (autonomousStep == 2){
            return AutoCodeLines.getVelocityX(secondPosition, thirdPosition, s);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getVelocityX(thirdPosition, endPosition, s);
        }
        return 0;
    }

    @Override
    public double velocityFunctionY(double s, double time) {
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityY(startPosition, secondPosition, s);
        }
        if (autonomousStep == 2){
            return AutoCodeLines.getVelocityY(secondPosition, thirdPosition, s);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getVelocityY(thirdPosition, endPosition, s);
        }
        return 0;
    }

    @Override
    public double velocityFunctionTheta(double s, double time) {
        return 0;
    }

    @Override
    public double getS(double time) {
        return time*ds;
    }
}
