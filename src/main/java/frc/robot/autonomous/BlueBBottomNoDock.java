package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

// Blue B1A No Dock
// Out 190, up 7, out 12, pick, back 12, up 19, back 190

public class BlueBBottomNoDock extends genericAutonomous{
    double xspd = 0;
    double yspd = 0;
    double turnspd = 0;
    double chasispseed = 0;
    double radius = 0.5;
    private final Timer m_timer = new Timer();
    @Override
    public void autonomousInit(GenericRobot robot) {
        super.autonomousInit(robot);
        //reset values
        m_timer.reset();
        autonomousStep = 0;
    }
    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        super.autonomousPeriodic(robot);
        SmartDashboard.putNumber("autostep", autonomousStep);
        Pose2d currPose = robot.getPose();
        switch(autonomousStep){
            // reset case
            case 0:
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                autonomousStep ++;
                break;
            // drop preloaded
            case 1:
                autonomousStep++;
                break;
            case 2:
                double t = m_timer.get();
                /*double s_0 = getS(t);
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
                }*/
                double s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                if(s_0 >= 190) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    autonomousStep++;
                }
                break;
            case 3:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = 0;
                yspd = velocityFunctionY(s_0);
                turnspd = 0;
                if(s_0 >= 7) {
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
                yspd = 0;
                turnspd = 0;
                if(s_0 >= 202) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    autonomousStep++;
                }
                break;
            case 5:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = -velocityFunctionX(s_0);
                yspd = 0;
                turnspd = 0;
                if(s_0 <= 190) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    autonomousStep++;
                }
                break;
            case 6:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = 0;
                yspd = velocityFunctionY(s_0);
                turnspd = 0;
                if(s_0 <= )
        }
        robot.setDrive(xspd, yspd, turnspd);
    }

    @Override
    public double positionFunctionX(double s) {
        return super.positionFunctionX(s);
    }

    @Override
    public double positionFunctionY(double s) {
        return super.positionFunctionY(s);
    }

    @Override
    public double positionFunctionTheta(double s) {
        return super.positionFunctionTheta(s);
    }

    @Override
    public double velocityFunctionX(double s) {
        return super.velocityFunctionX(s);
    }

    @Override
    public double velocityFunctionY(double s) {
        return super.velocityFunctionY(s);
    }

    @Override
    public double velocityFunctionTheta(double s) {
        return super.velocityFunctionTheta(s);
    }

    @Override
    public double getS(double time) {
        return super.getS(time);
    }


}
