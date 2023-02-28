package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class FiveFeetRightTwo extends genericAutonomous{

    private final Timer m_timer = new Timer();
    double firstDist = 60;
    double secondDist = 60;
    double radius = 30;
    double desiredTheta = 5*Math.PI/2;
    double desiredInchesPerSecond = 12;
    double ds = desiredInchesPerSecond;
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
                xspd = 1*ds;
                // + xPidK*(positionFunctionX(s_0) - currPose.getX());
                yspd = 0;
                //  + yPidK*(positionFunctionY(s_0) - currPose.getY());
                turnspd = 0;

                if (s_0 >= firstDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    autonomousStep += 1;
                }
                break;
            case 2:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = 0;
                yspd = 1*ds;
                turnspd = 0;
                if (s_0 >= firstDist+secondDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.stop();
                    autonomousStep++;
                }
                break;
            case 3:
                xspd = 0;
                yspd = 0;
                turnspd = 0;
        }
        robot.setDrive(xspd, yspd, turnspd);
    }


    @Override
    public double positionFunctionX(double s){
        if (s <= firstDist){
            return s;
        }
        else if (s <= firstDist + radius*desiredTheta){
            return radius*Math.sin((s-firstDist)/radius);
        }
        else{
            return firstDist+radius;
        }
    }

    @Override
    public double positionFunctionY(double s){
        if (s <= firstDist){
            return 0;
        }
        else if (s <= firstDist + radius*desiredTheta){
            return -radius*Math.cos((s-firstDist)/radius);
        }
        else{
            double a = firstDist + radius*desiredTheta;
            double c = radius;
            return  (c + s - a);
        }
    }
    @Override
    public double positionFunctionTheta(double s){
        return 0;
    }
    @Override
    public double velocityFunctionX(double s,double time){
        if (s <= firstDist){
            return 1*ds;
        }
        else if (s <= firstDist + secondDist){
            return 0;
        }
        else{
            return 0;
        }
    }
    @Override
    public double velocityFunctionY(double s,double time){
        if (s <= firstDist){
            return 0;
        }
        else if (s <= firstDist + secondDist){
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
    public double velocityFunctionTheta(double s,double time){
        return 0;
    }
    @Override
    public double getS(double time){
        return time*ds;
    }


}
