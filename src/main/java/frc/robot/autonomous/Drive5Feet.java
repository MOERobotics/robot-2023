package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.swerveBot;

public class Drive5Feet extends genericAutonomous {
    Pose2d current_position;
    int autonomousStep = 0;

    double start_distance;
    double default_power;
    double start_time;
    double xspd = .5;
    double yspd = .5;
    double rspd = 0;
    @Override
    public void autonomousInit(GenericRobot robot){
        autonomousStep = 0;
        start_time = System.currentTimeMillis();
        //TODO: add pid stuff
    }
    @Override
    public void autonomousPeriodic(GenericRobot robot){
        switch(autonomousStep){
            case 0: //reset
                if(System.currentTimeMillis() - start_time > 100){
                    autonomousStep += 1;
                }
                break;
            case 1: //drive forward 5ft
                autonomousStep += 1;
                break;
            case 2: //stop

                break;
        }
    }
    @Override
    public void trajectoryCalc(GenericRobot robot, Pose2d origin, Pose2d dest) {
        Pose2d diff = diff(robot, origin, dest);
        double dx = diff.getX();
        double dy = diff.getY();
        xspd = (dx != dxran) ? 0.5 : 0;
        yspd = (dy != dyran) ? 0.5 : 0;
        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(xspd, yspd, rspd);
        SwerveModuleState[] moduleStates = robot.kinematics().toSwerveModuleStates(targetChassisSpeeds);
        SwerveModuleState frontLeftState = moduleStates[0],
                frontRightState = moduleStates[1],
                backLeftState = moduleStates[2],
                backRightState = moduleStates[3];

        frontLeftState = SwerveModuleState.optimize(frontLeftState, Rotation2d.fromDegrees(robot.getPivotLeftMotorA()));
        frontRightState = SwerveModuleState.optimize(frontRightState, Rotation2d.fromDegrees(robot.getPivotRightMotorA()));
        backLeftState = SwerveModuleState.optimize(backLeftState, Rotation2d.fromDegrees(robot.getPivotLeftMotorB()));
        backRightState = SwerveModuleState.optimize(backRightState, Rotation2d.fromDegrees(robot.getPivotRightMotorB()));

        robot.swerve(frontLeftState, frontRightState, backLeftState, backRightState);
    }

    @Override
    public Pose2d diff(GenericRobot robot, Pose2d one, Pose2d two) {
             return new Pose2d(one.getX()-two.getX(), one.getY()-two.getY(), Rotation2d.fromDegrees(0));
    }
}
