package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class FFTRPiecewise extends genericAutonomous{

    private final Timer m_timer = new Timer();
    double firstDist = 60;
    double secondDist = 60;
    double radius = 30;
    double desiredTheta = 5*Math.PI/2;
    double desiredInchesPerSecond = 12;
    double ds = desiredInchesPerSecond;
    double xspd, yspd, turnspd;
    double t;
    double velCalcT;
    double t1;

    double baseSpd;
    double s_0;
    double robotWidth;
    @Override
    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        autonomousStep = 0;
        robot.resetAttitude();
        robot.resetPIDPivot();
        robot.resetStartHeading();
        robot.resetStartDists();
        robot.resetStartPivots();

    }
    @Override
    public void autonomousPeriodic(GenericRobot robot){
        //I4H
        SmartDashboard.putNumber("autostep", autonomousStep);
        SmartDashboard.putNumber("s",s_0);
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
                t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                if(s_0>= firstDist+secondDist) {
                    autonomousStep++;
                }
                break;
            case 2:
                xspd = yspd = turnspd = 0;
                break;
        }
        robot.setDrive(xspd, yspd, turnspd);
    }


    @Override
    public double positionFunctionX(double s){
        return 0;
    }

    @Override
    public double positionFunctionY(double s){
        return 0;
    }
    @Override
    public double positionFunctionTheta(double s){
        return 0;
    }
    @Override
    public double velocityFunctionX(double s){
        if(s<=firstDist){
            return 1*ds;
        } else{
            return 0;
        }
    }
    @Override
    public double velocityFunctionY(double s){
        if(s<=firstDist){
            return 0;
        } else if(s<= firstDist+secondDist){
            return -1*ds;
        } else{
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
