package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import org.opencv.core.Point;


public class E2Dock extends genericAutonomous{

    private final Timer m_timer = new Timer();
    boolean sensorTripped = false;
    Point startPosition = new Point(55.8, 55.8);
    Point secondPosition = new Point(207.7-27, 55.8);
    Point thirdPosition = new Point(276.88,85.6);
    //276.88

    double distanceOne = AutoCodeLines.getDistance(startPosition,secondPosition)+4;
    double distanceTwo = AutoCodeLines.getDistance(secondPosition, thirdPosition);

    double desiredInPerSec = 40;
    double ds = desiredInPerSec;
    double xspd,yspd,turnspd, s;

    double xPidK = 3;
    double yPidK = 3;
    double firstDist = 120;
    double secondDist = 60;
    double radius = 30;
    double desiredTheta = 5*Math.PI/2;
    double distance =0;
    double currDistance;
    double x1,x2,y1,y2;
    double travelTime;
    double dist;
    double t;
    double velCalcT;
    double t1;
    boolean passedFulcrum;
    boolean resetPose;

    double baseSpd;
    double s_0;
    double robotWidth;
    boolean resetting = false;
    double currPitch;
    double currRoll;
    double curPosOnRamp;
    double currPosInAutoBalance;

    double leftside;
    double rightside;
    int autoStepback;
    int autoSequenceStep;
    double currentpos;
    double initPos;
    double startingPose;
    double desiredPitch = 9.0;
    double totalPathLength = 0;
    double initpos;
    double boundPos1;
    double boundPos2;
    double boundPos3;
    double kP = 0.05;
    double lengthOfField = 450.1;
    PIDController PID = new PIDController(kP,0,0);
    @Override
    public void autonomousInit(GenericRobot robot){
        sensorTripped = false;
        m_timer.reset();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetStartHeading();
        if(robot.getRed()){
            yspd = -yspd;
            robot.setPose(new Pose2d(lengthOfField - startPosition.x, startPosition.y, new Rotation2d(0)));
        } else{
            robot.setPose(new Pose2d(startPosition.x, startPosition.y, new Rotation2d(0)));
        }
        robot.setPose(new Pose2d(startPosition.x, startPosition.y, new Rotation2d(0)));
        autonomousStep = 0;
    }
    @Override
    public void autonomousPeriodic(GenericRobot robot){
        baseSpd = desiredInPerSec;
        double correctionPower = -14.0;
        double climbPower = -30.0;
        double basePower = -35.0;
        currPitch = robot.getPitch();
        currRoll = robot.getRoll();
        //I4H
        SmartDashboard.putNumber("autostep", autonomousStep);
        SmartDashboard.putNumber("s",s_0);
        SmartDashboard.putNumber("xpos", robot.getPose().getX());
        SmartDashboard.putNumber("ypos", robot.getPose().getY());
        SmartDashboard.putBoolean("LightSensor tripped", sensorTripped);
        Pose2d currPose = robot.getPose();
        switch(autonomousStep){
            case 0:
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = s = 0;
                autonomousStep++;
                break;
            case 1:
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s);
                yspd = velocityFunctionY(s);
                turnspd = velocityFunctionTheta(s);
                if (s >= distanceOne){
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                //xspd = velocityFunctionX(0);
                //yspd = velocityFunctionY(0);
                /*if(Math.abs(robot.getPitch())<=2){
                    passedFulcrum = true;
                }
                if(passedFulcrum && Math.abs(robot.getPitch())>2){
                    resetPose = true;
                }
                Pose2d offTheStation = new Pose2d(151.9+robotWidth,55.8, Rotation2d.fromDegrees(0));
                if(robot.getPitch() <=2&&resetPose){
                    robot.setPose(offTheStation);
                    passedFulcrum = false;
                    resetPose = false;
                    autonomousStep++;
                }*/
                break;
            case 2: //on cs
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s);
                yspd = velocityFunctionY(s);
                turnspd = velocityFunctionTheta(s);
                if(robot.getRightLightSensor()){
                    sensorTripped = true;
                    robot.setPose(new Pose2d(secondPosition.x, secondPosition.y, new Rotation2d(0)));
                } else if(robot.getLeftLightSensor()){
                    sensorTripped = true;
                    robot.setPose(new Pose2d(secondPosition.x, secondPosition.y, new Rotation2d(0)));
                }
                if (s >= distanceTwo){
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                //currDistance = Math.sqrt((x1-robot.getPose().getX())*(x1-robot.getPose().getX()) + (y1-robot.getPose().getY())*(y1-robot.getPose().getY()));
                //xspd = velocityFunctionX(0);
                //yspd = velocityFunctionY(0);
                //if(distance <=currDistance) {
                //autonomousStep++;
                //}
                /*
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s) + xPidK*(positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s) + yPidK*(positionFunctionY(s) - currPose.getY());
                turnspd = velocityFunctionTheta(s);
                if (s >= distanceOne){
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
                */
                break;
            case 3:
                /*t = m_timer.get();
                dist = y2-y1;
                travelTime = dist/baseSpd;
                if(t-t1 <= travelTime){
                    xspd = 0;
                    yspd = baseSpd;
                } else {
                    xspd = yspd = turnspd = 0;
                    autonomousStep++;
                }*/
                xspd = yspd = turnspd = 0;
                autonomousStep++;
                break;
            case 4:
                xspd = basePower;
                if (Math.abs(currPitch) > 11) { // driving back up charge station
                    autonomousStep += 1;
                }
                break;
            case 5:
                xspd = climbPower;
                if (Math.abs(currPitch) < 10) { //flattened out
                    autonomousStep++;
                }
                break;
            case 6:
                if (currPitch < -desiredPitch) { //correcting begins
                    xspd = correctionPower; //backward
                } else if (currPitch > desiredPitch) {
                    xspd = -correctionPower; //forward
                } else {
                    xspd = 0;
                }
                break;
        }
        if (autonomousStep > 0) turnspd = PID.calculate(-robot.getYaw());
        robot.setDrive(xspd, -yspd, turnspd);
    }


    @Override
    public double positionFunctionX(double s){
        if (autonomousStep == 0){
            return startPosition.x*ds;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getPositionX(startPosition, secondPosition, s)*ds;
        }
        if (autonomousStep == 2){
            return AutoCodeLines.getPositionX(secondPosition, thirdPosition, s)*ds;
        }
        return thirdPosition.x;
    }

    @Override
    public double positionFunctionY(double s){
        if (autonomousStep == 0){
            return startPosition.y*ds;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getPositionY(startPosition, secondPosition, s)*ds;
        }
        if (autonomousStep == 2){
            return AutoCodeLines.getPositionY(secondPosition, thirdPosition, s)*ds;
        }
        return thirdPosition.y;

    }
    @Override
    public double positionFunctionTheta(double s){
        return 0;
    }
    @Override
    public double velocityFunctionX(double s){
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityX(startPosition, secondPosition, s)*ds;
        }
        if (autonomousStep == 2){
            return AutoCodeLines.getVelocityX(secondPosition, thirdPosition, s)*ds;
        }
        return 0;
    }
    @Override
    public double velocityFunctionY(double s){
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityY(startPosition, secondPosition, s)*ds;
        }
        if (autonomousStep == 2){
            return AutoCodeLines.getVelocityY(secondPosition, thirdPosition, s)*ds;
        }
        return 0;
    }
    @Override
    public double velocityFunctionTheta(double s){
        return 0;
    }
    @Override
    public double getS(double time){
        return time*ds;
    }


}