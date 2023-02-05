package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class Blue_I_Bottom_NoDock_I4H extends genericAutonomous{

    private final Timer m_timer = new Timer();
    double firstDist = 48;
    double secondDist = 48;
    double radius = 30;
    double desiredTheta = 5*Math.PI/2;
    double desiredInchesPerSecond = 12;
    double ds = desiredInchesPerSecond;
    double xspd, yspd, turnspd;
    double t;

    double basePower;
    double s_0;
    double robotWidth;
    @Override
    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        autonomousStep = 0;

    }
    @Override
    public void autonomousPeriodic(GenericRobot robot){
        //I4H
        SmartDashboard.putNumber("autostep", autonomousStep);
        Pose2d currPose = robot.getPose();
        switch(autonomousStep){
            case 0:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                robot.setDrive(xspd,0,0);
                autonomousStep++;
                break;
            case 1:
                t = m_timer.get();
                if(getS(t) == 220-s_0-robotWidth){
                    robot.setDrive(0,0,0);
                }
                break;
            case 2:
                t = m_timer.get();
                break;
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
    public double velocityFunctionX(double s){
        if (s <= firstDist){
            return 1*ds;
        }
        else if (s <= firstDist + radius*desiredTheta){
            return radius*Math.cos((s-firstDist)/radius)*(1/radius)*ds;
        }
        else{
            return 0;
        }
    }
    @Override
    public double velocityFunctionY(double s){
        if (s <= firstDist){
            return 0;
        }
        else if (s <= firstDist + radius*desiredTheta){
            return radius*Math.sin((s-firstDist)/radius)*(1/radius)*ds;
        }
        else{
            double a = firstDist + radius*desiredTheta;
            double c = radius;
            return  1*ds;
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
