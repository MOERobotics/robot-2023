package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class DriveInASquare extends genericAutonomous{

    private final Timer m_timer = new Timer();
    double firstDist = 60;
    double secondDist = 60;
    double thirdDist = 60;
    double fourthDist = 60;
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
                xspd = velocityFunctionX(s_0,t);
                // + xPidK*(positionFunctionX(s_0) - currPose.getX());
                yspd = velocityFunctionY(s_0,t);
                //  + yPidK*(positionFunctionY(s_0) - currPose.getY());
                turnspd = velocityFunctionTheta(s_0,t);
                if (s_0 >= firstDist + secondDist + thirdDist + fourthDist) {
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
    public double velocityFunctionX(double s, double time){
        if (s <= firstDist){
            return 1*ds;
        }
        else if (s <= firstDist + secondDist){
            return 0;
        }
        else if (s <= firstDist + secondDist + thirdDist) {
            return -1*ds;
        }
        else if (s <= firstDist + secondDist + thirdDist + fourthDist) {
            return 0;
        }
        else{
            return 0;
        }
    }

    @Override
    public double velocityFunctionY(double s, double time){
        if (s <= firstDist){
            return 0;
        }
        else if (s <= firstDist + secondDist){
            return -1*ds;
        }
        else if (s <= firstDist + secondDist + thirdDist) {
            return 0;
        }
        else if (s <= firstDist + secondDist + thirdDist + fourthDist) {
            return 1*ds;
        }
        else{
            return 0;
        }
    }

    @Override
    public double velocityFunctionTheta(double s, double time){
        return 0;
    }

    @Override
    public double getS(double time){
        return time*ds;
    }
}
