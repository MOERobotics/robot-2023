package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;


public class mobilityEngage extends genericAutonomous{
    double basePower = 35.0;
    double high = 13.5;
    double climbPower = 30.0;
    double correctionPower = 13.0;
    double desiredPitch = 9.0;
    double firstBreak = 5;
    double dropping = 9;
    double xspd;
    double timerDelta;
    Timer m_timer = new Timer();
    double xPos;
    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        timerDelta = 0;
        climbPower = 30;
        basePower = 35.0;
        correctionPower = 13.0;
        robot.setPigeonYaw(-90);
        robot.resetStartDists();
        robot.resetAttitude();
        robot.resetStartPivots();
        robot.resetStartHeading();
        robot.setPose();
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("autonomousStep", autonomousStep);

        double currPitch = -robot.getRoll();
        switch (autonomousStep) {
            case 0:
                xspd = basePower;
                if (Math.abs(currPitch) > firstBreak) {

                    //Add length of the robot from front encoder to end of back wheel.
                    autonomousStep++;
                }
                break;
            case 1:
                xspd = basePower;
                if (currPitch > high) {
                    xPos = robot.getPose().getX();
                    autonomousStep++;
                }
                break;
            case 2:
                xspd = climbPower;
                if (Math.abs(currPitch) < dropping) {
                    autonomousStep++;
                }
                break;
            case 3:
                xspd = climbPower+4;
                if (Math.abs(currPitch) > high){
                    autonomousStep ++;
                }
                break;
            case 4:
                xspd = climbPower+4;
                if (Math.abs(currPitch) < desiredPitch){
                    xPos = robot.getPose().getX();
                    autonomousStep ++;
                }
                break;
            case 5:
                xspd = climbPower+4;
                if (Math.abs(robot.getPose().getX() - xPos) > 20){
                    xspd = 0;
                    autonomousStep ++;
                }
                break;
            case 6:
                climbPower = -30;
                basePower = -35.0;
                correctionPower = 13.0;
                autonomousStep ++;
                break;
            case 7:
                xspd = basePower;
                if (Math.abs(currPitch) > high) {
                    xPos = robot.getPose().getX();
                    autonomousStep++;
                }
                break;
            case 8:
                if(Math.abs(robot.getPose().getX() - xPos) >= 32){
                    climbPower = -13;
                }
                xspd = climbPower;
                if (Math.abs(currPitch) < dropping) {
                    autonomousStep++;
                }
                break;
            case 9:
                xspd = correctionPower;
                m_timer.reset();
                m_timer.start();
                //This is a future feature to stop and let others get on before autobalancing.
                autonomousStep++;
                break;
            case 10:
                if ((currPitch < -desiredPitch) && (m_timer.get() <1)) { //correcting begins
                    timerDelta = m_timer.get();
                    xspd = -correctionPower; //backward
                } else if ((currPitch > desiredPitch) && (m_timer.get() <1)) {
                    timerDelta = m_timer.get();
                    xspd = correctionPower;//forward
                } else {
                    xspd = 0;
                    if (m_timer.get()-timerDelta > .25) {
                        m_timer.reset();
                        m_timer.start();
                    }
                }
                break;
        }
        if (robot.getPose().getX() > 200){
            xspd = 0;
        }
        robot.setDrive(xspd, 0, 0, true);
    }
}
