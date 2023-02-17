package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import org.opencv.core.Point;

public class VisionScoring extends genericAutonomous{

    Point startPosition = new Point(610.74,0 );
    Point endPosition = new Point(610.74, 36);

    double distanceOne = AutoCodeLines.getDistance(startPosition,endPosition);
    double desiredInPerSec = 25;
    double ds = desiredInPerSec;
    double xspd,yspd,turnspd, s;

    double xPidK = 1.0e-3;
    double yPidK = 1.0e-3;
    private final Timer m_timer = new Timer();
    PIDController yawControl = new PIDController(.5, 0,0);

    @Override
    public void autonomousInit(GenericRobot robot) {
        yawControl.enableContinuousInput(-180,180);
        yawControl.reset();
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
                xspd = velocityFunctionX(s) + xPidK*(positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s) + yPidK*(positionFunctionY(s) - currPose.getY());
                turnspd = velocityFunctionTheta(s);
                if (s >= distanceOne){
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
            case 2:
                m_timer.reset();
                xspd = yspd = turnspd = 0;
                break;
        }
        turnspd = yawControl.calculate(-robot.getYaw());
        robot.setDrive(xspd,yspd,turnspd);
    }

    @Override
    public double positionFunctionX(double s) {
        if (autonomousStep == 0){
            return startPosition.x;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getPositionX(startPosition, endPosition, s);
        }
        return endPosition.x;
    }

    @Override
    public double positionFunctionY(double s) {
        if (autonomousStep == 0){
            return startPosition.y;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getPositionY(startPosition, endPosition, s);
        }
        return endPosition.y;
    }

    @Override
    public double positionFunctionTheta(double s) {
        return 0;
    }

    @Override
    public double velocityFunctionX(double s) {
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityX(startPosition, endPosition, s)*ds;
        }
        return 0;
    }

    @Override
    public double velocityFunctionY(double s) {
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityY(startPosition, endPosition, s)*ds;
        }
        return 0;
    }

    @Override
    public double velocityFunctionTheta(double s) {
        return 0;
    }

    @Override
    public double getS(double time) {
        return time*ds;
    }
}
