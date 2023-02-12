package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class A1B2CnoDock extends genericAutonomous{
    private final Timer m_timer = new Timer();

    double distAto1 = 208.13;
    double dist1toB = 207.18;
    double distBto2 = 216.70;
    double dist2toC = 221.155;

    double inchesPerSecond = 20;
    double ds = inchesPerSecond;
    double xspd, yspd, turnspd;

    @Override
    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        autonomousStep = 0;
    }
    @Override
    public void autonomousPeriodic(GenericRobot robot){
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
                if (s_0 >= distAto1 + dist1toB + distBto2 + dist2toC) {
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
        if (s <= distAto1){
            return s;
        }
        else if (s <= distAto1 + dist1toB){
            return s;
        }
        else{
            return distAto1 + dist1toB;
        }
    }

    @Override
    public double positionFunctionY(double s){
        if (s <= distAto1){
            return 0;
        }
        else if (s <= distAto1 + dist1toB){
            return 0;
            //return -radius*Math.cos((s-firstDist)/radius);
        }
        else{
            /*double a = firstDist + radius*desiredTheta;
            double c = radius;
            return  (c + s - a);*/
            return 0;
        }
    }
    @Override
    public double positionFunctionTheta(double s){
        return 0;
    }
    @Override
    public double velocityFunctionX(double s){
        if (s <= distAto1){
            return 1*ds;
        }
        else if (s <= distAto1 + dist1toB){
            return 1*ds;
        }
        else if (s <= distAto1 + dist1toB + distBto2){
            return 1*ds;
        }
        else if (s <= distAto1 + dist1toB + distBto2 + dist2toC){
            return 1*ds;
        }
        else{
            return 0;
        }
    }
    @Override
    public double velocityFunctionY(double s){
        if (s <= distAto1){
            return 0;
        }
        else if (s <= distAto1 + dist1toB){
            return 1*ds;
        }
        else if (s <= distAto1 + dist1toB + distBto2){
            return 1*ds;
        }
        else if (s <= distAto1 + dist1toB + distBto2 + dist2toC){
            return 1*ds;
        }
        else{
            /*double a = firstDist + radius*desiredTheta;
            double c = radius;
            return  1*ds;*/
            return 0;
        }
    }
    @Override
    public double velocityFunctionTheta(double s){
        return 0;
    }
    @Override
    public double getS(double time){
        return time*ds;
    }
}
