package frc.robot.autonomous;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.generic.GenericRobot;
import frc.robot.vision.Detection;
import frc.robot.vision.MoeNetVision;

public class ObjectDetectionTest extends genericAutonomous {
    final double TARGET_DISTANCE = 12.;
    MoeNetVision vision;
    double xPosStar, yPosStar, xspd, yspd, turnspd;
    double defaultSpeed = 10;
    Pose2d currPose = null;
    Pose2d desiredPose;
    @Override
    public void autonomousInit(GenericRobot robot) {
        vision = new MoeNetVision(robot);
        if (vision.poseFound()){
            currPose = new Pose2d();
        }
        xPosStar = currPose.getX();
        yPosStar = currPose.getY();

    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        ///////////////////////////////////////////////////find robot pose on field
        currPose = robot.getPose();
        double xPos = currPose.getX();
        double yPos = currPose.getY();

        switch (autonomousStep){
            case 0: {
                Detection firstDetection = vision.firstObjectDetection();
                Pose2d robotTargetPosition = null;

                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                                    .times(Units.metersToInches(1));
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);

                    this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    autonomousStep = 1;
                }

                xspd = yspd = turnspd = 0;
                break;
            }
            case 1:
                double xDiff = desiredPose.getX()-xPos;
                double yDiff = desiredPose.getY()-yPos;
                double totDiff = Math.hypot(xDiff, yDiff);
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