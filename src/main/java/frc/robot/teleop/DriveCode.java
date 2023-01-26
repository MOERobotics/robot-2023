package frc.robot.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class DriveCode extends GenericTeleop{
    double[] startDists;
    double[] startPivots;
    double startHeading;
    double oldLeftA = 0;
    double oldLeftB = 0;
    double oldRightA = 0;
    double oldRightB = 0;

    boolean resetting = false;
    Joystick swerveStick = new Joystick(1);

    @Override
    public void teleopInit(GenericRobot robot) {
        resetting = false;
        robot.setOffsetLeftA();
        robot.setOffsetLeftB();
        robot.setOffsetRightA();
        robot.setOffsetRightB();

        robot.resetAttitude();
        robot.resetPIDPivot();

        startHeading = robot.getYaw();

        startDists = new double[] {robot.getDriveDistanceInchesLeftA(), robot.getDriveDistanceInchesRightA(),
                robot.getDriveDistanceInchesLeftB(),robot.getDriveDistanceInchesRightB()};

        startPivots = new double[] {robot.getPivotLeftMotorA(), robot.getPivotRightMotorA(),
                robot.getPivotLeftMotorB(), robot.getPivotRightMotorB()};
    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {

        if (resetting) robot.resetAttitude();
        if (!resetting || (resetting && Math.abs(robot.getYaw()) < 1)) {
            resetting = false;
////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
            Pose2d robotPose = robot.getPose(startHeading, robot.getYaw(), startDists, startPivots, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
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
            }

            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xspd,
                    yspd, turnspd, Rotation2d.fromDegrees(-robot.getYaw()));
            SwerveModuleState[] moduleStates = robot.kinematics().toSwerveModuleStates(chassisSpeeds);
            SwerveModuleState frontLeftState = moduleStates[0],
                    frontRightState = moduleStates[1],
                    backLeftState = moduleStates[2],
                    backRightState = moduleStates[3];

            frontLeftState = robot.optimizeSwervePivots(frontLeftState, Rotation2d.fromDegrees(robot.getPivotLeftMotorA()));
            frontRightState = robot.optimizeSwervePivots(frontRightState, Rotation2d.fromDegrees(robot.getPivotRightMotorA()));
            backLeftState = robot.optimizeSwervePivots(backLeftState, Rotation2d.fromDegrees(robot.getPivotLeftMotorB()));
            backRightState = robot.optimizeSwervePivots(backRightState, Rotation2d.fromDegrees(robot.getPivotRightMotorB()));

            if (xspd == 0 && yspd == 0 && turnspd == 0) {
                robot.stopSwerve(oldLeftA, oldRightA, oldLeftB, oldRightB);
            } else {
                robot.swerve(frontLeftState, frontRightState, backLeftState, backRightState);
                oldLeftA = frontLeftState.angle.getDegrees();
                oldLeftB = backLeftState.angle.getDegrees();
                oldRightA = frontRightState.angle.getDegrees();
                oldRightB = backRightState.angle.getDegrees();
            }

            if (swerveStick.getRawButton(1)) {
                resetting = true;
                robot.setOffsetLeftA();
                robot.setOffsetLeftB();
                robot.setOffsetRightA();
                robot.setOffsetRightB();

                robot.resetAttitude();
                robot.resetPIDPivot();

                startHeading = robot.getYaw();

                startDists = new double[]{robot.getDriveDistanceInchesLeftA(), robot.getDriveDistanceInchesRightA(),
                        robot.getDriveDistanceInchesLeftB(), robot.getDriveDistanceInchesRightB()};

                startPivots = new double[]{robot.getPivotLeftMotorA(), robot.getPivotRightMotorA(),
                        robot.getPivotLeftMotorB(), robot.getPivotRightMotorB()};
            }
        }
        SmartDashboard.putBoolean("I am resetting", resetting);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////end swerve code



    }


}
