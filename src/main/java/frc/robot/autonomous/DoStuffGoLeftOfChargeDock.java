package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class DoStuffGoLeftOfChargeDock extends genericAutonomous {

    private final Timer m_timer = new Timer();

    double baseSpd = 25;
    double firstDist = 208.11;
    double secondDist = 220.08;
    double thirdDist = 220.75;
    double fourthDist = 221.76;
    double fifthDist = 63.43;
    double t = m_timer.get();
    double desiredInchesPerSecond = 12;
    double ds = desiredInchesPerSecond;
    double xspd, yspd, turnspd = 0;
    double startTime;

    @Override
    public void autonomousInit(GenericRobot robot) {
        Pose2d currPose = robot.getPose();
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
            switch (autonomousStep) {
                case 0:
                    double t = m_timer.get();
                    double s_0 = getS(t);
                    xspd = yspd = turnspd = 0;
                    autonomousStep++;
                    break;
                case 1:
                    t = m_timer.get();
                    s_0 = getS(t);
                    xspd = velocityFunctionX(s_0);
                    yspd = velocityFunctionY(s_0);
                    if (s_0 > firstDist) {
                        xspd = (220)/firstDist;
                        yspd = 182.235;
                        turnspd = 0;
                        autonomousStep++;
                    }
                    break;
                case 2:
                    t = m_timer.get();
                    s_0 = getS(t);
                    xspd = velocityFunctionX(s_0);
                    yspd = velocityFunctionY(s_0);
                    if (s_0 > firstDist + secondDist) {
                        xspd = 0;
                        yspd = 0;
                        turnspd = 0;
                        autonomousStep++;
                    }
                    break;
                case 3:
                    t = m_timer.get();
                    s_0 = getS(t);
                    xspd = velocityFunctionX(s_0);
                    yspd = velocityFunctionX((s_0));
                    if (s_0 > firstDist + secondDist + thirdDist) {
                        xspd = 0;
                        yspd = 0;
                        turnspd = 0;
                        autonomousStep++;
                    }
                    break;
                case 4:
                    t = m_timer.get();
                    s_0 = getS(t);
                    xspd = velocityFunctionX(s_0);
                    yspd = velocityFunctionY(s_0);
                    if (s_0 > firstDist + secondDist + thirdDist + fourthDist) {
                        xspd = 0;
                        yspd = 0;
                        turnspd = 0;
                        m_timer.stop();
                    }
                    break;
                case 5:
                    t = m_timer.get();
                    s_0 = getS(t);
                    if (s_0 > firstDist + secondDist + thirdDist + fourthDist + fifthDist){
                        xspd = 0;
                        yspd = 0;
                        turnspd = 0;
                        m_timer.stop();
                    }
            }
        }

    @Override
    public double velocityFunctionY(double s) {
        if (s <= firstDist) {
            return 0;
        } else if (s <= firstDist + secondDist) {
            return ds * Math.sin(-1 * Math.PI / 3);
        } else {
            return 0;
        }
    }
    @Override
    public double velocityFunctionX(double s){
        if (s <= firstDist){
            return 1*ds;
        }
        else if (s <= firstDist + secondDist){
            return ds*Math.cos(-1*Math.PI/3);
        }
        else{
            return 0;
        }
    }
}

