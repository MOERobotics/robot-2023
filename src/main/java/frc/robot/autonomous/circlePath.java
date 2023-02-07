package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class circlePath extends genericAutonomous{

    private final Timer m_timer = new Timer();
    double radius = 30;
    double desiredInPerSec = 40;
    double ds = desiredInPerSec;
    double xspd, yspd, turnspd;
    double increment = .01;

    double xPidK = .05;
    double yPidK = .05;
    @Override
    public void autonomousInit(GenericRobot robot) {
        m_timer.reset();
        autonomousStep = 0;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("autostep", autonomousStep);
        Pose2d currPose = robot.getPose();
        switch(autonomousStep){
            case 0:
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                autonomousStep ++;
                break;
            case 1:
                double t = m_timer.get();
                double s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                       // + xPidK*(positionFunctionX(s_0) - currPose.getX());
                yspd = velocityFunctionY(s_0);
                      //  + yPidK*(positionFunctionY(s_0) - currPose.getY());
                turnspd = velocityFunctionTheta(s_0);
                if (s_0 >= 6*Math.PI*radius) { //1 full circle
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.stop();
                    autonomousStep += 1;
                }
                break;
            case 2:
                xspd = 0;
                yspd = 0;
                turnspd = 0;
        }
        robot.setDrive(xspd, yspd, turnspd);
    }

    @Override
    public double positionFunctionX(double s){
        return radius*Math.cos(s/radius);
    }

    @Override
    public double positionFunctionY(double s){
        return radius*Math.sin(s/radius);
    }

    @Override
    public double positionFunctionTheta(double s){
        return 0;
    }

    @Override
    public double velocityFunctionX(double s){
        return -radius*Math.sin(s/radius)*ds/radius;
    }

    @Override
    public double velocityFunctionY(double s){
        return radius*Math.cos(s/radius)*ds/radius;
    }

    @Override
    public double velocityFunctionTheta(double s){
        return 0;
    }

    @Override
    public double getS(double t){
        return t*ds;
    }
}