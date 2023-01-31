package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.generic.GenericRobot;

public class DriveInASquare extends genericAutonomous {
    int autonomousStep = 0;

    //drive_distance is in inches
    double drive_distance = 12;
    double turning_speed = 2.5;
    double starting_pose = 0;
    Pose2d start_position = new Pose2d(0,0, Rotation2d.fromDegrees(0));
    Pose2d destination_step1 = new Pose2d(0,drive_distance, Rotation2d.fromDegrees(0));
    Pose2d destination_step2 = new Pose2d(-drive_distance,0, Rotation2d.fromDegrees(0));
    Pose2d destination_step3 = new Pose2d(0, -drive_distance, Rotation2d.fromDegrees(0));
    Pose2d destination_step4 = new Pose2d(drive_distance, 0, Rotation2d.fromDegrees(0));
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
            case 1: //drive forward
                trajectoryCalc(robot,start_position,destination_step1);
                autonomousStep += 1;
                break;
            case 2: //stop the momentum
                robot.stopSwerve(0,0,0,0);
                autonomousStep += 1;
                break;
            case 3: //drive to the left
                trajectoryCalc(robot,destination_step1,destination_step2);
                autonomousStep += 1;
                break;
            case 4: //stop the momentum
                robot.stopSwerve(0,0,0,0);
                autonomousStep += 1;
                break;
            case 5: //drive backwards
                trajectoryCalc(robot,destination_step2,destination_step3);
                autonomousStep += 1;
                break;
            case 6: //stop the momentum
                robot.stopSwerve(0,0,0,0);
                starting_pose = destination_step3.getRotation().getDegrees();
                robot.setDrive(0,0,turning_speed);
                autonomousStep += 1;
                break;
            case 7: //turn 90 to the right
                if(destination_step3.getRotation().getDegrees() <= starting_pose-90){
                    robot.setDrive(0,0,0);
                }
                autonomousStep += 1;
                break;
            case 8: //drive to original position
                trajectoryCalc(robot,destination_step3,destination_step4);
                autonomousStep += 1;
                break;
            case 9: //stop the momentum
                robot.stopSwerve(0,0,0,0);
                starting_pose = destination_step3.getRotation().getDegrees();
                robot.setDrive(0,0,-2.5);
                autonomousStep += 1;
                break;
            case 10: //turn 270 to the right
                if(destination_step3.getRotation().getDegrees() <= 90 && destination_step3.getRotation().getDegrees() > 0){
                    robot.setDrive(0,0,0);
                }
                autonomousStep += 1;
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
