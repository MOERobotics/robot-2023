package frc.robot.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autoBalance;
import frc.robot.commands.autoBalanceBackward;
import frc.robot.commands.genericCommand;
import frc.robot.generic.GenericRobot;


public class DriveCode extends GenericTeleop{

    boolean resetting = false;
    Joystick xbox = new Joystick(1);

    //currently this is the joystick
    Joystick xbox2 = new Joystick(0);

    double currPitch;
    double currRoll;
    double curPosOnRamp;
    double currPosInAutoBalance;

    double leftside;
    double rightside;
    int autoStep;
    int autoSequenceStep;
    double currentpos;
    double initPos;
    double autoStepback;
    double startingPose;
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
    boolean initialize = false;
    boolean initialize1 = false;

    genericCommand balance = new autoBalance();
    genericCommand balanceBack = new autoBalanceBackward();

    //////////////////////////////////////////////////////////////////////////////////////////////////Arm Code Constants
    double collectorRPM = 0;
    double armPower = 0;
    boolean dropTopRoller = false;
    boolean openGripper = false;

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
            double xspd = robot.deadzone(-swerveStick.getRawAxis(1), .35) * robot.getMaxInchesPerSecond() / 2;
            double yspd = robot.deadzone(-swerveStick.getRawAxis(0), .35) * robot.getMaxInchesPerSecond() / 2;
            double turnspd = robot.deadzone(-swerveStick.getRawAxis(4), .35) * robot.getMaxRadPerSec() / 2;

            currPitch = robot.getPitch(); //test switching roll and pitch
            currRoll = robot.getRoll();

            //TODO: Delete later, added to push code
            SmartDashboard.putNumber("poseX", robotPose.getX());
            SmartDashboard.putNumber("poseY", robotPose.getY());
            SmartDashboard.putNumber("poseZ", robotPose.getRotation().getDegrees());
            SmartDashboard.putNumber("bound1", boundPos1);
            SmartDashboard.putNumber("bound2", boundPos2);
            SmartDashboard.putNumber("bound3", boundPos3);
            SmartDashboard.putNumber("Pitch", robot.getPitch());
            SmartDashboard.putNumber("Roll", robot.getRoll());
            SmartDashboard.putNumber("Yaw", robot.getYaw());
            SmartDashboard.putNumber("xspd", xspd);
            SmartDashboard.putNumber("yspd", yspd);
            SmartDashboard.putNumber("turnspd", turnspd);
            SmartDashboard.putNumber("autostep", autoStep);
            SmartDashboard.putNumber("boundPos1", boundPos1);
            SmartDashboard.putNumber("boundPos2", boundPos2);
            SmartDashboard.putNumber("boundPos3", boundPos3);
            SmartDashboard.putNumber("autoStepback", autoStepback);
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

        SmartDashboard.putBoolean("I am resetting", resetting);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////end swerve code
        // Bumpers left 5, right 6

        if (xbox2.getRawButton(5)){ //move collector up
            dropTopRoller = false;
        }
        else if (xbox2.getRawButton(6)){ //move collector down
            dropTopRoller = true;
        }

        // 2 is b, 3 is x

        if (xbox2.getRawButton(3)){ //collect in
            collectorRPM = 10000;
        }
        else if (xbox2.getRawButton(2)){ //collect out
            collectorRPM = -10000;
        }
        else{ //no more collecting :(
            collectorRPM = 0;
        }
        //TODO: change the getRawButton to triggers, left trigger barfs the piece out, right trigger


        //currently using Joystick buttons

        //gripper functions currently do not work.
        if(xbox2.getRawButton(13)){ //open gripper?
            openGripper = true;
        }
        else if (xbox2.getRawButton(12)) { //close gripper?
            openGripper = false;
        }

        armPower = -robot.deadzone(xbox2.getRawAxis(1), .2);

        ///////////////////////////////////////////////////////////////////////////Power setters
        robot.collect(collectorRPM);
        robot.raiseTopRoller(dropTopRoller);
        robot.moveArm(armPower);
        robot.openGripper(openGripper);
    }
}