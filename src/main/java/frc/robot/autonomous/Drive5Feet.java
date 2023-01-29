package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.generic.GenericRobot;

public class Drive5Feet extends genericAutonomous {
    int autonomousStep = 0;

    Pose2d start_position = new Pose2d(0,0,Rotation2d.fromDegrees(0));
    Pose2d destination = new Pose2d(0,60,Rotation2d.fromDegrees(0));
    double xspd = .5;
    double yspd = .5;
    double rspd = 0;
    @Override
    public void autonomousInit(GenericRobot robot){
        autonomousStep = 0;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){
        switch(autonomousStep){
            case 0: //reset
                robot.SwerveAutoReset();
                autonomousStep += 1;
                break;
            case 1: //drive forward 5ft
                trajectoryCalc(robot,start_position,destination);
                autonomousStep += 1;
                break;
            case 2: //stop
                robot.stopSwerve(0,0,0,0);
                break;
        }
    }
    @Override
    public void trajectoryCalc(GenericRobot robot, Pose2d origin, Pose2d dest) {
        Pose2d diff = diff(robot, origin, dest);
        double dx = diff.getX();
        double dy = diff.getY();
        Rotation2d dr = diff.getRotation();
        xspd = (dx != dest.getX()) ? 0.5 : 0;
        yspd = (dy != dest.getY()) ? 0.5 : 0;
        rspd = (dr != dest.getRotation()) ? 0 : 0;
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
             return new Pose2d(one.getX()-two.getX(), one.getY()-two.getY(), Rotation2d.fromDegrees(Math.atan((two.getY()-one.getY())/(two.getX()-one.getX()))));
    }

    @Override
    public Pose2d getNextPose(GenericRobot robot, Pose2d one, double dx, double dy) {
        return new Pose2d((int) (dx/one.getX())+dx%one.getX(), (int) (dy/one.getY())+dy%one.getY(), Rotation2d.fromDegrees(Math.atan((dy-one.getY())/(dx-one.getX()))));
    }


}
