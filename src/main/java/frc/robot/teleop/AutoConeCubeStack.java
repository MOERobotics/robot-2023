package frc.robot.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import org.opencv.core.Point;

public class AutoConeCubeStack extends GenericTeleop{

    Joystick xbox = new Joystick(1); //TODO: change to a button box later on
    double[] nodeYPositions = {19.5,45.1, 66.4, 89.3, 111.54, 133.43, 156.26, 176.91, 201.02};
    double xVal = 53.64+14;
    double topNodeConeHeight = 46;
    double midNodeConeHeight = 34;
    double topCubeHeight = 35.5;
    double midCubeHeight = 23.5;
    double[] armPositions = {topNodeConeHeight, midNodeConeHeight, topCubeHeight, midCubeHeight};


    Point bottLeftCS = new Point(117.84-20, 59.37-20);
    Point topRightCS = new Point(191.83+20, 154.94+20);

    double defaultSpeed = 24;

    Point desiredPoint;
    Pose2d desiredPose;
    double xspd,yspd, turnspd, armPower;
    int nodeYPos, nodeZPos;
    boolean openGripper = false;
    @Override
    public void teleopInit(GenericRobot robot) {
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetStartHeading();
        robot.setPose();
        xspd = yspd = turnspd = armPower = 0;
    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {
        Pose2d currPose = robot.getPose();
        if (xbox.getRawButtonPressed(2)){ //For testing purposes. This will be 3rd station, mid-Cone
            nodeYPos = 2;
            nodeZPos = 2;
            openGripper = false;
            desiredPoint = new Point(xVal, nodeYPositions[nodeYPos]);
            desiredPose = new Pose2d(desiredPoint.x, desiredPoint.y, new Rotation2d(0));
        }

        xspd = AutoCodeLines.getVelocityX(new Point(currPose.getX(), currPose.getY()), desiredPoint, 1);
        yspd = AutoCodeLines.getVelocityY(new Point(currPose.getX(), currPose.getY()), desiredPoint, 1);

        if (currPose.getX() >= bottLeftCS.x && currPose.getX() <= topRightCS.x
        && currPose.getY() >= bottLeftCS.y && currPose.getY() <= topRightCS.y){
            //check which side I'm on
            if (Math.abs(currPose.getX() - bottLeftCS.x) <= 6){
                //top line
                //same as before no change
            }
            else if (Math.abs(currPose.getY() - topRightCS.y) <= 6){
                xspd = -defaultSpeed;
                yspd = 0;
            }
            else if (Math.abs(currPose.getY() - bottLeftCS.y) <= 6){
                xspd = -defaultSpeed;
                yspd = 0;
            }
            else{
                //bott line
                if (Math.abs(currPose.getY() - bottLeftCS.y) + Math.abs(desiredPose.getY()- bottLeftCS.y) <
                        Math.abs(currPose.getY() - topRightCS.y) + Math.abs(desiredPose.getY()- topRightCS.y)){
                    yspd = -defaultSpeed;
                }
                else{
                    yspd = defaultSpeed;
                }
                xspd = 0;
            }
        }

        if (Math.abs(currPose.getX() - desiredPose.getX()) < 3 && Math.abs(currPose.getY() - desiredPose.getY()) < 3){
            xspd = yspd = turnspd = 0;
            armPower = .2;
            if (Math.abs(robot.getArmPosition() - armPositions[nodeZPos]) <= 3){
                armPower = 0;
                openGripper = true;
            }
        }

        robot.setDrive(xspd,yspd,turnspd,true);
        robot.moveArm(armPower);
        robot.openGripper(openGripper);

    }
}
