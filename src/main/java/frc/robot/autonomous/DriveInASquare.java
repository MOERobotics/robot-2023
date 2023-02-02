package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generic.GenericRobot;

public class DriveInASquare extends genericAutonomous {
    int autonomousStep = 0;
    double xPower = 0;
    double yPower = 0;
    double turnPower = 0; //in radians

    double startingPose;
    double length = 36; //in inches
    double arclength = 90; //in degrees

    double noPower = 0;
    double defaultPower = 5; //in inches
    double defaultTurnPower = 1; //in radians


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
                robot.resetStartPivots();
                robot.resetStartDists();
                robot.resetStartHeading();

                startingPose = robotPose.getX();
                xPower = defaultPower;
                autonomousStep++;
                break;
            case 1:
                if(robotPose.getX() >= startingPose+length){
                    xPower = noPower;
                    startingPose = robotPose.getY();
                    //TODO: might have to change to negative
                    yPower = defaultPower;
                    autonomousStep++;
                }
                break;
            case 2:
                if(robotPose.getY()<= startingPose-length){
                    yPower = noPower;
                    startingPose = robotPose.getX();
                    xPower = -defaultPower;
                    autonomousStep++;
                }
                break;
            case 3: //Travel to ___ and start turning 90 degrees to the left
                if(robotPose.getX()<= startingPose-length){
                    xPower = noPower;
                    startingPose = robotPose.getRotation().getDegrees();
                    turnPower = -defaultTurnPower;
                    autonomousStep++;
                }
                break;
            case 4:
                if(robotPose.getRotation().getDegrees()<=startingPose-arclength){
                    turnPower = noPower;
                    startingPose = robotPose.getY();
                    yPower = defaultPower;
                    autonomousStep++;
                }
                break;
            case 5:
                if(robotPose.getY()<= startingPose+length){
                    yPower = noPower;
                    startingPose = robotPose.getRotation().getDegrees();
                    turnPower = -defaultTurnPower;
                    autonomousStep++;
                }
                break;
            case 6:
                if(robotPose.getRotation().getDegrees() <= arclength && robotPose.getRotation().getDegrees() > 0){
                    turnPower = noPower;
                    autonomousStep++;
                }
                break;

        }

        robot.setDrive(xPower, yPower, turnPower);
    }
}
