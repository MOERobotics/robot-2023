package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

import static frc.robot.Robot.driveCode;

public class autoRoutine extends genericAutonomous{
    double xspd, yspd, turnspd;
    double radius = 30;
    double desiredTheta = 5*Math.PI/2;
    double desiredInchesPerSecond = 12;
    double ds = desiredInchesPerSecond;
    int autoStep;
    double firstDist = 208.11;
    double secondDist = 220.08;
    double thirdDist = 220.75;
    double fourthDist = 221.76;
    double fifthDist = 63.43;
    double baseSpd = 25.0;


    private final Timer m_timer = new Timer();

    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        autoStep = 0;
    }
    public void teleopPeriodic(GenericRobot robot) {
        driveCode.teleopPeriodic(robot);
    }
    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        SmartDashboard.putNumber("autostep", autoStep);
        Pose2d currPose = robot.getPose();
        switch(autoStep){
            case 0:
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                autoStep ++;
            case 1:
                double t = m_timer.get();
                double s_0 = getS(t);
                xspd = ((275.88-55.88)/firstDist)*baseSpd;
                yspd = ((182.235-200.47)/firstDist)*baseSpd;
                if(s_0 >= firstDist){
                    xspd = 0;
                    yspd = 0;
                    autoStep++;
                }
                break;
            case 2:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = ((55.88-275.88)/firstDist+secondDist)*baseSpd;
                yspd = ((200.47-182.235)/firstDist+secondDist)*baseSpd;

                if(s_0 >= (secondDist)/baseSpd) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    autoStep++;
                }
                break;
            case 3:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = 0;
                yspd = ((154.37-200.47)/firstDist+secondDist+thirdDist)*baseSpd;
                if(s_0 >= thirdDist/baseSpd) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    autoStep++;
                }
                break;
            case 4://right chariging board
                t = m_timer.get();
                s_0 = getS(t);
                xspd = ((115.7-55.88)/firstDist+secondDist+thirdDist+fourthDist)*baseSpd;
                yspd = ((131.81-154.37)/firstDist+secondDist+thirdDist+fourthDist)*baseSpd;
                if(s_0 >= fourthDist/baseSpd) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.stop();
                    autoStep++;
                }
                break;


                //balance thing here



        }
        robot.setDrive(xspd,yspd,turnspd);
    }

    public double velocityFunctionX(double s){
        double x1 = 0;
        double y1 = 0;
        double x2 = 0;
        double y2 = 0;
        double m;
        /*if (s <= firstDist){
            x1 = 208.11;
            y1=55.88;
            x2 = 428.86;
            y2=275.88;
            m = (y2 - y1)/(x2-x1);
            return m;
        }
        else if (s <= firstDist + secondDist){
            x2 = 845.06;
            y2=55.88;
            x1 = 428.86;
            y1=275.88;
            m = (y2 - y1)/(x2-x1);
            return m;
        }
        else if (s <= firstDist + secondDist+thirdDist){
            x1 = 845.06;
            y1=55.88;
            x2 = 891.16;
            y2=55.88;
            m = (y2 - y1)/(x2-x1);
            return m;
        }else if (s <= firstDist + secondDist+thirdDist+fourthDist){
            x2 = 845.06;
            y2=55.88;
            x1 = 954.59;
            y1=115.7;
            m = (y2 - y1)/(x2-x1);
        }else if (s <= firstDist + secondDist+thirdDist+fourthDist+fifthDist){
            return .94;
        } */
        return 0;
    }
    @Override
    public double velocityFunctionY(double s){
     /*   if (s <= firstDist){

            return -.08;
        }
        else if (s <= firstDist + secondDist){
            return .08;
        }
        else if (s <= firstDist + secondDist+thirdDist){
            return -.21;
        }else if (s <= firstDist + secondDist+thirdDist+fourthDist){
            return -1;
        }else if (s <= firstDist + secondDist+thirdDist+fourthDist+fifthDist){
            return .94;
        } */
        return 0;
    }
}
