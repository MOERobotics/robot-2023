package frc.robot.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class DriveCode extends GenericTeleop{

    boolean resetting = false;
    Joystick xbox = new Joystick(1);
    Joystick xbox2 = new Joystick(0);

    double currPitch;
    double currRoll;
    double curPosOnRamp;
    double currPosInAutoBalance;

    double leftside;
    double rightside;
    int autoStep;
    double currentpos;
    double initPos;
    double desiredPitch = 9.0;
    double initpos;
    double boundPos1;
    double boundPos2;
    double boundPos3;

    double currentHead;
    double error;
    double correction;
    double desiredAngle;
    double startingPos;

    double startAngle;
    boolean btnLeft = false;
    boolean btnRight = false;
    boolean autoBalance;
    int count;
    double totalPathLength = 0;

    //////////////////////////////////////////////////////////////////////////////////////////////////Arm Code Constants
    double collectorRPM = 0;
    double armPower = 0;
    boolean liftTopRoller = false;

    @Override
    public void teleopInit(GenericRobot robot) {
        resetting = false;
        robot.setOffsetLeftA();
        robot.setOffsetLeftB();
        robot.setOffsetRightA();
        robot.setOffsetRightB();

        robot.resetAttitude();
        robot.resetPIDPivot();

        robot.resetStartHeading();

        robot.resetStartDists();

        robot.resetStartPivots();
        robot.setPose();
    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {


        if (resetting){
            robot.resetAttitude();
            robot.setPose();
        }
        if (!resetting || (resetting && Math.abs(robot.getYaw()) < 1)) {
            resetting = false;
////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
            Pose2d robotPose = robot.getPose();

////////////////////////////////////////////////////////////////////////////////////////////////////////////Swerve code
            double xspd = robot.deadzone(-xbox.getRawAxis(1), .35) * robot.getMaxInchesPerSecond() / 2;
            double yspd = robot.deadzone(-xbox.getRawAxis(0), .35) * robot.getMaxInchesPerSecond() / 2;
            double turnspd = robot.deadzone(-xbox.getRawAxis(4), .35) * robot.getMaxRadPerSec() / 2;

            if (xbox.getRawButton(5)) { // for varun
                turnspd *= 2;
            }
            if (xbox.getRawButton(6)) {
                xspd *= 2;
                yspd *= 2;
                SmartDashboard.putNumber("xspd", xspd);
            }

            robot.setDrive(xspd, yspd, turnspd);

            if (xbox.getRawButton(1)) {
                resetting = true;
                robot.setOffsetLeftA();
                robot.setOffsetLeftB();
                robot.setOffsetRightA();
                robot.setOffsetRightB();

                robot.resetAttitude();
                robot.resetPIDPivot();

                robot.resetStartHeading();

                robot.resetStartDists();

                robot.resetStartPivots();
            }
        }
        SmartDashboard.putBoolean("I am resetting", resetting);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////end swerve code
        // Bumpers left 5, right 6

        if (xbox2.getRawButton(5)){ //move roller up and down
            liftTopRoller = true;
        }
        else if (xbox2.getRawButton(6)){
            liftTopRoller = false;
        }

        // 2 is b, 3 is x

        if (xbox2.getRawButton(3)){ //collect in
            collectorRPM = 10000;
        }
        else if (xbox2.getRawButton(2)){ //collect out
            collectorRPM = -10000;
        }
        else{
            collectorRPM = 0;
        }

        armPower = robot.deadzone(xbox2.getRawAxis(1), .2);

        ///////////////////////////////////////////////////////////////////////////Power setters
        robot.collect(collectorRPM);
        robot.raiseTopRoller(liftTopRoller);
        robot.moveArm(armPower);


    }


}
