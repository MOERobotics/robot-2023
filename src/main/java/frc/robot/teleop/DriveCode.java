package frc.robot.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.generic.GenericRobot;

public class DriveCode extends GenericTeleop{

    Joystick swerveStick = new Joystick(1);

    @Override
    public void teleopInit(GenericRobot robot) {
        robot.resetAttitude();
        robot.resetPIDPivot();
    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {
////////////////////////////////////////////////////////////////////////////////////////////////////////////Swerve code
        double xspd = robot.deadzone(-swerveStick.getRawAxis(1), .35)*robot.getMaxMeterPerSec();
        double yspd = robot.deadzone(-swerveStick.getRawAxis(0), .35)*robot.getMaxMeterPerSec();
        double turnspd = robot.deadzone(-swerveStick.getRawAxis(4), .35)*robot.getMaxRadPerSec();

        if (swerveStick.getRawButton(4)){ // for varun
            turnspd /= 2;
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xspd,
                yspd, turnspd, Rotation2d.fromDegrees(-robot.getYaw()));
        SwerveModuleState[] moduleStates = robot.kinematics().toSwerveModuleStates(chassisSpeeds);
        SwerveModuleState frontLeftState = moduleStates[0],
                frontRightState = moduleStates[1],
                backLeftState = moduleStates[2],
                backRightState = moduleStates[3];

        frontLeftState = SwerveModuleState.optimize(frontLeftState, Rotation2d.fromDegrees(robot.getPivotLeftMotorA()));
        frontRightState = SwerveModuleState.optimize(frontRightState, Rotation2d.fromDegrees(robot.getPivotRightMotorA()));
        backLeftState = SwerveModuleState.optimize(backLeftState, Rotation2d.fromDegrees(robot.getPivotLeftMotorB()));
        backRightState = SwerveModuleState.optimize(backRightState, Rotation2d.fromDegrees(robot.getPivotRightMotorB()));

        if (xspd == 0 && yspd == 0 && turnspd == 0){
            robot.stopSwerve();
        }
        else{
            robot.swerve(frontLeftState, frontRightState, backLeftState, backRightState);
        }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////end swerve code



    }


}
