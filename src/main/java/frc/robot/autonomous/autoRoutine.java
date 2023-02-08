package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class autoRoutine extends genericAutonomous{
    double xspd, yspd, turnspd;
    int autoStep;
    double firstDist = 208.11;
    double secondDist = 220.08;
    double thirdDist = 220.75;
    double fourthDist = 221.76;
    double fifthDist = 63.43;
    double baseSpd;


    private final Timer m_timer = new Timer();

    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        autoStep = 0;
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
                break;
            case 1:
                double t = m_timer.get();
                double s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                if(s_0 >= firstDist){
                    xspd = 0;
                    yspd = 0;
                    autoStep++;
                }

                break;
            case 2:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                if(s_0 >= firstDist + secondDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    autoStep++;
                }
                break;
            case 3:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                if(s_0 >= firstDist + secondDist+thirdDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    autoStep++;
                }
                break;
            case 4:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = 0;
                if(s_0 >= firstDist + secondDist+thirdDist+fourthDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    autoStep++;
                }
                break;
            case 5: //right of charging board
                t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                if(s_0 >= firstDist + secondDist+thirdDist+fourthDist+fifthDist) {
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


}
