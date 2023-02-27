package frc.robot.autonomous;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.vision.Detection;
import frc.robot.vision.MoeNetVision;

public class ObjectDetectionTest extends genericAutonomous {
    final double TARGET_DISTANCE = 12.;
    MoeNetVision vision;
    double defaultSpeed = 20;
    Pose2d currPose = null;
    Pose2d desiredPose;
    @Override
    public void autonomousInit(GenericRobot robot) {
        vision = new MoeNetVision(robot);
        currPose = new Pose2d();

        robot.resetAttitude();
        robot.resetPIDPivot();
        robot.resetStartHeading();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.setPose();

        SmartDashboard.putString("detautoTarget", "none yet");
        desiredPose = null;
        autonomousStep = 0;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        ///////////////////////////////////////////////////find robot pose on field
        currPose = robot.getPose();
        double xPos = currPose.getX();
        double yPos = currPose.getY();

        double xspd = 0, yspd = 0, turnspd = 0;

        SmartDashboard.putString("detautoPosition", String.format("%f, %f", this.currPose.getX(), this.currPose.getY()));

        switch (autonomousStep){
            case 0: {
                Detection firstDetection = vision.firstObjectDetection();

                SmartDashboard.putBoolean("detautoHasFirstDetection", firstDetection != null);

                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                                    .times(Units.metersToInches(1));
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);

                    this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    SmartDashboard.putString("detautoTarget", String.format("%f, %f", this.desiredPose.getX(), this.desiredPose.getY()));
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