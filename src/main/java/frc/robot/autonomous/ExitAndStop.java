package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class ExitAndStop extends genericAutonomous{
    double defaultPower = 30.0;
    double xspd, yspd, turnspd;
    double xPos;
    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        defaultPower = 0;
        robot.resetStartDists();
        robot.resetAttitude();
        robot.resetStartPivots();
        robot.resetStartHeading();
        robot.setPose();
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("autonomousStep", autonomousStep);

        switch (autonomousStep) {
            case 0:
                xspd = defaultPower;
                yspd = turnspd = 0;
                xPos = robot.getPose().getX();
                if (xPos > 30) {
                    //Add length of the robot from front encoder to end of back wheel.
                    autonomousStep++;
                }
                break;
            case 1:
                xspd = 0;
                break;
        }
        if (robot.getPose().getX() > 200){
            xspd = 0;
        }
        robot.setDrive(xspd, yspd, turnspd, true, true);
    }
}