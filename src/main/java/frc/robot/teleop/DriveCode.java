package frc.robot.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class DriveCode extends GenericTeleop{

    boolean resetting = false;
    Joystick swerveStick = new Joystick(1);

    double yawDelta = 0;

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
    double kP = 0.5e-1;
    double desiredYaw = 0;
    PIDController yawControl = new PIDController(kP, 0,0);

    @Override
    public void teleopInit(GenericRobot robot) {
        yawControl.enableContinuousInput(-180,180);

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
            desiredYaw = robot.getYaw();
        }
        if (!resetting || (resetting && Math.abs(robot.getYaw()) < 1)) {
            resetting = false;
////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
            Pose2d robotPose = robot.getPose();

////////////////////////////////////////////////////////////////////////////////////////////////////////////Swerve code
            double xspd = robot.deadzone(-swerveStick.getRawAxis(1), .35) * robot.getMaxInchesPerSecond() / 2;
            double yspd = robot.deadzone(-swerveStick.getRawAxis(0), .35) * robot.getMaxInchesPerSecond() / 2;
            double turnspd = robot.deadzone(-swerveStick.getRawAxis(4), .35) * robot.getMaxRadPerSec() / 2;

            if (swerveStick.getRawButton(5)) { // for varun
                turnspd *= 2;
            }
            if (swerveStick.getRawButton(6)) {
                xspd *= 2;
                yspd *= 2;
                SmartDashboard.putNumber("xspd", xspd);
            }
            if (turnspd != 0){
                desiredYaw = robot.getYaw();
            }
            else {
                turnspd = yawControl.calculate(desiredYaw-robot.getYaw());
            }

            robot.setDrive(xspd, yspd, turnspd);

            if (swerveStick.getRawButton(1)) {
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


    }


}
