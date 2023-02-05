package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//This drives in a rectangular path
public class SlendermanPants extends genericAutonomous {
    private final Timer m_timer = new Timer();
    double firstDist = 190;
    double secondDist = 10;
    double thirdDist = 190;
    double fourthDist = 0;
    double radius = 30;
    double desiredTheta = 5*Math.PI/2;
    double desiredInchesPerSecond = 35;
    double ds = desiredInchesPerSecond;
    double xspd, yspd, turnspd;
    double startTime;

    @Override
    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetStartHeading();
        startTime = System.currentTimeMillis();
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
                if(System.currentTimeMillis() - startTime >= 500) {
                    autonomousStep++;
                }
                break;
            case 1:
                double t = m_timer.get();
                double s_0 = getS(t);
                xspd = vFX(s_0, autonomousStep);
                yspd = vFY(s_0, autonomousStep);
                if (s_0 >= firstDist) {
                    xspd = 0;
                    yspd = 0;
                    autonomousStep ++;
                }
                break;
            case 2:
                xspd = 0;
                yspd = 0;
                turnspd = 0;
                if(System.currentTimeMillis() - startTime >= 500) {
                    autonomousStep++;
                }
                break;
            case 3:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = vFX(s_0, autonomousStep);
                yspd = vFY(s_0, autonomousStep);
                if (s_0 >= firstDist + secondDist) {
                    xspd = 0;
                    yspd = 0;
                    autonomousStep ++;
                }
                break;
            case 4:
                xspd = 0;
                yspd = 0;
                turnspd = 0;
                double zero = 0;
                if(System.currentTimeMillis() - startTime >= 500) {
                    autonomousStep++;
                }
                break;
            case 5:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = vFX(s_0, autonomousStep);
                yspd = vFY(s_0, autonomousStep);
                if (s_0 >= firstDist + secondDist + thirdDist) {
                    xspd = 0;
                    yspd = 0;
                    m_timer.stop();
                    autonomousStep ++;
                }
                break;
            case 6:
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
        else if (s <= firstDist + secondDist){
            return 0;
        }
        else if (s <= firstDist + secondDist + thirdDist) {
            return -s;
        }
        else if (s <= firstDist + secondDist + thirdDist + fourthDist) {
            return 0;
        }
        else{
            return 0;
        }
    }

    @Override
    public double positionFunctionY(double s){
        if (s <= firstDist){
            return 0;
        }
        else if (s <= firstDist + secondDist){
            return -s;
        }
        else if (s <= firstDist + secondDist + thirdDist) {
            return 0;
        }
        else if (s <= firstDist + secondDist + thirdDist + fourthDist) {
            return s;
        }
        else{
            return 0;
        }
    }

    @Override
    public double positionFunctionTheta(double s){
        return 0;
    }

    @Override
    public double vFX(double s, int autoStep){
        if (autoStep == 1) {
            if (s <= firstDist){
                return 1*ds;
            }
        }
        else if (autoStep == 3) {
            if (s <= firstDist + secondDist){
                return 0;
            }
        }
        else if (autoStep == 5) {
            if (s <= firstDist + secondDist + thirdDist) {
                return -1*ds;
            }
        }
        else{
            return 0;
        }
        return 0;
    }

    @Override
    public double vFY(double s, int autoStep){
        if (autoStep == 1) {
            if (s <= firstDist){
                return 0;
            }
        }
        else if (autoStep == 3) {
            if (s <= firstDist + secondDist){
                return -1*ds;
            }
        }
        else if (autoStep == 5) {
            if (s <= firstDist + secondDist + thirdDist) {
                return 0;
            }
        }
        else{
            return 0;
        }
        return 0;
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
