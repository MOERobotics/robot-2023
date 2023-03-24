package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
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
        Pose2d currPose = robot.getPose();
        switch (autonomousStep) {
            case 0:
                xspd = defaultPower;
                if (robot.getRed()) xspd *= -1;
                xPos = currPose.getX();
                yspd = turnspd = 0;
                if (Math.abs(xPos - currPose.getX()) >= 60) {
                    xspd = 0;
                    autonomousStep++;
                }
            case 1:
                xspd = 0;
                break;
        }
        robot.setDrive(xspd, yspd, turnspd, true, true);
    }
}