package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.generic.GenericRobot;
import frc.robot.vision.MoeNetVision;

public class visionTestPose extends genericAutonomous {

    MoeNetVision vision;
    Field2d field = new Field2d();
    double xPosStar, yPosStar, xspd, yspd, turnspd;
    double defaultSpeed = 10;
    Pose2d currPose = null;
    Pose2d desiredPose;
    @Override
    public void autonomousInit(GenericRobot robot) {
        vision = new MoeNetVision(robot);
        if (vision.poseFound()){
            currPose = vision.robotFieldPoseInches();
            xPosStar = field.getRobotPose().getX();
            yPosStar = field.getRobotPose().getY();
        }

    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        ///////////////////////////////////////////////////find robot pose on field

        if (vision.poseFound()){
            currPose = vision.robotFieldPoseInches();
        }
        double xPos = currPose.getX();
        double yPos = currPose.getY();

        switch (autonomousStep){
            case 0:
                xspd = yspd = turnspd = 0;
                if (vision.poseFound()){
                    xPosStar = vision.robotFieldPoseInches().getX();
                    yPosStar = vision.robotFieldPoseInches().getY();
                    desiredPose = new Pose2d(xPosStar + 12, yPosStar + 12,
                            new Rotation2d(0));
                    autonomousStep ++;
                }
                break;
            case 1:
                double xDiff = desiredPose.getX()-xPos;
                double yDiff = desiredPose.getY()-yPos;
                double totDiff = Math.sqrt(xDiff*xDiff + yDiff*yDiff);
                xspd = defaultSpeed * xDiff/totDiff;
                yspd = defaultSpeed * yDiff/totDiff;
                if (Math.abs(xDiff) <= .5 && Math.abs(yDiff) <= .5){
                    xspd = yspd = 0;
                    autonomousStep ++;
                }
                break;
            case 2:
                xspd = yspd = turnspd = 0;
                break;
        }
        robot.setDrive(xspd, yspd, turnspd);
    }


}