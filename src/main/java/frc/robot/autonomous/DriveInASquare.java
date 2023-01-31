package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.generic.GenericRobot;

public class DriveInASquare extends genericAutonomous {
    int autonomousStep = 0;

    double startingPose;
    double drive_distance = 12;
    double turning_speed = 2.5;
    double starting_pose = 0;
    double basePower = 35;
    double turnPower = 2.5;
    double length = 12;
    Pose2d robotPose;

    @Override
    public void autonomousInit(GenericRobot robot){
        robotPose = robot.getPose();
        autonomousStep = 0;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){
        switch(autonomousStep){
            case 0: //reset
                robot.SwerveAutoReset();
                startingPose = robotPose.getX();
                autonomousStep += 1;
                break;
            case 1:
                if(robotPose.getX() >= startingPose+length){
                    robot.setDrive(0,0,0);
                    autonomousStep++;
                }
                break;
            case 2:
                startingPose = robotPose.getY();
                robot.setDrive(0,basePower, 0);
                autonomousStep++;
                break;
            case 3:
                if(robotPose.getY()<= startingPose-length){
                    robot.setDrive(0,0,0);
                    autonomousStep++;
                }
                break;
            case 4:
                startingPose = robotPose.getX();
                robot.setDrive(-basePower,0,0);
                autonomousStep++;
                break;
            case 5:
                if(robotPose.getX()<= startingPose-length){
                    robot.setDrive(0,0,0);
                    autonomousStep++;
                }
                break;
            case 6:
                startingPose = robotPose.getRotation().getDegrees();
                robot.setDrive(0,0,turnPower);
                autonomousStep++;
                break;
            case 7:
                if(robotPose.getRotation().getDegrees()<=startingPose-90){
                    robot.setDrive(0,0,0);
                    autonomousStep++;
                }
                break;
            case 8:
                startingPose = robotPose.getY();
                robot.setDrive(0,basePower,0);
                autonomousStep++;
                break;
            case 9:
                if(robotPose.getY()<= startingPose+12){
                    robot.setDrive(0,0,0);
                    autonomousStep++;
                }
                break;
            case 10:
                startingPose = robotPose.getRotation().getDegrees();
                robot.setDrive(0,0,-turnPower);
                autonomousStep++;
                break;
            case 11:
                if(robotPose.getRotation().getDegrees() <=90 && robotPose.getRotation().getDegrees() > 0){
                    robot.setDrive(0,0,0);
                    autonomousStep++;
                }
                break;
        }
    }
}
