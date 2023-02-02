package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.vision.MoeNetVision;

public class visionTestPose extends genericAutonomous {

    MoeNetVision vision = new MoeNetVision(NetworkTableInstance.getDefault());
    Field2d field = new Field2d();
    double xPosStar, yPosStar, xspd, yspd, turnspd;
    double defaultSpeed = 10;
    @Override
    public void autonomousInit(GenericRobot robot) {
        Pose3d currPose = vision.getPose();
        if (vision.poseFound()){
            field.setRobotPose(currPose.toPose2d());
        }
        xPosStar = field.getRobotPose().getX();
        yPosStar = field.getRobotPose().getY();
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        ///////////////////////////////////////////////////find robot pose on field
        Pose3d currPose = vision.getPose();
        if (vision.poseFound()){
            field.setRobotPose(currPose.toPose2d());
        }
        double xPos = field.getRobotPose().getX();
        double yPos = field.getRobotPose().getY();

        Pose2d desiredPose = new Pose2d(new Translation2d(xPosStar + 12, yPosStar + 12),
                new Rotation2d(0));

        switch (autonomousStep){
            case 0:
                xspd = yspd = turnspd = 0;
                autonomousStep ++;
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