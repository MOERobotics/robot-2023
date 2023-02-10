package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class overStationAndBalance extends genericAutonomous{

    private final Timer m_timer = new Timer();
    double firstDist = 120;
    double secondDist = 60;
    double radius = 30;
    double desiredTheta = 5*Math.PI/2;
    double distance =0;
    double desiredInchesPerSecond = 12;
    double ds = desiredInchesPerSecond;
    double xspd, yspd, turnspd;
    double currDistance;
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
    @Override
    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        autonomousStep = 0;
        robot.resetAttitude();
        robot.resetPIDPivot();
        robot.resetStartHeading();
        robot.resetStartDists();
        robot.resetStartPivots();

    }
    @Override
    public void autonomousPeriodic(GenericRobot robot){
        baseSpd = 25.0;
        double correctionPower = -14.0;
        double climbPower = -30.0;
        double basePower = -35.0;
        currPitch = robot.getRoll(); //test switching roll and pitch
        currRoll = robot.getPitch();
        //I4H
        SmartDashboard.putNumber("autostep", autonomousStep);
        SmartDashboard.putNumber("s",s_0);
        Pose2d currPose = robot.getPose();
        switch(autonomousStep){
            case 0:
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                autonomousStep++;
                break;
            case 1:
                t = m_timer.get();
                distance = 151.9;
                double x2 = 207.7;
                double x1 = 55.8;
                double y1 = 55.8;
                double y2 = 276.8;
                xspd = velocityFunctionX(0);
                yspd = velocityFunctionY(0);
                if(Math.abs(robot.getPitch())<=2){
                    passedFulcrum = true;
                }
                if(passedFulcrum && Math.abs(robot.getPitch())>2){
                    resetPose = true;
                }
                Pose2d offTheStation = new Pose2d(151.9,55.3, Rotation2d.fromDegrees(0));
                if(robot.getPitch() <=2&&resetPose){
                    robot.setPose(offTheStation);
                    passedFulcrum = false;
                    resetPose = false;
                    autonomousStep++;
                }
                break;
            case 2:
                distance = 33.1;
                x1 = 207.7;
                x2 = 221.0;
                y1 = 55.8;
                y2 = 276.8;
                currDistance = Math.sqrt((x1-robot.getPose().getX())*(x1-robot.getPose().getX()) + (y1-robot.getPose().getY())*(y1-robot.getPose().getY()));
                xspd = velocityFunctionX(0);
                yspd = velocityFunctionY(0);
                if(distance <=currDistance) {
                    autonomousStep++;
                }
                break;
            case 3:
                xspd = yspd = turnspd = 0;
                autonomousStep++;
                break;
            case 4:
                xspd = basePower;
                if (Math.abs(currPitch) > 5) {
                    autonomousStep++;
                }
                break;
            case 5:
//                        curPosOnRamp = base * Math.sin(currPitch) * (Math.cos(angleOfBoard) / Math.sin(angleOfBoard));
//                        leftside = basePower*(base - curPosOnRamp)/base;
//                        rightside = basePower*(base - curPosOnRamp)/base;
                xspd = basePower;
                if (Math.abs(currPitch) > 11) {
                    autonomousStep += 1;
                }
                break;
            case 6:
                xspd = climbPower;
                if (Math.abs(currPitch) < 10) {
                    autonomousStep++;
                }
                break;
            case 7:
                //initPos = robotPose.getX() ;
                xspd = -correctionPower;
                //This is a future feature to stop and let others get on before autobalancing.
                    /*if(Math.abs(curPitch)<15){
                        leftside = 0;
                        rightside = 0;
                    }
                    if(leftJoystick.getRawButtonPressed(9)) {
                        autoStep++;
                    }*/
                autonomousStep++;
                break;
                /*case 4:
                    currentpos = robotPose.getY() ;
                    leftside = -climbPower;
                    rightside = -climbPower;
                    if(Math.abs(initPos - currentpos) > 1){
                        leftside = 0;
                        rightside = 0;
                        autoStep++;
                    }
                    break;
                case 5:
                    leftside = 0;
                    rightside = 0;
                    if(Math.abs(currPitch) < 2){
                        autoStep++;
                    }
                    break;*/
            case 8:
                if (currPitch < -desiredPitch) {
                    xspd = -correctionPower;
                } else if (currPitch > desiredPitch) {
                    xspd = correctionPower;
                } else {
                    xspd = 0;
                    autonomousStep++;
                }
                break;
            case 9:
                if (Math.abs(currPitch) > desiredPitch) {
                    autonomousStep--;
                } else {
                    xspd = yspd = turnspd = 0;
                }
                break;
        }
        robot.setDrive(xspd, yspd, turnspd);
    }


    @Override
    public double positionFunctionX(double s){
        if(autonomousStep == 1) {
            distance = 151.9;
            double x2 = 207.7;
            double x1 = 55.8;
            return 0;
        } else if( autonomousStep == 2){
            distance = 33.1;
            double x1 = 207.7;
            double x2 = 221.0;
            return 0;
        }
        else{
            return 0;
        }

    }

    @Override
    public double positionFunctionY(double s){
        if(autonomousStep == 1 || autonomousStep == 2){
            distance = 221.0;
            double y1 = 55.8;
            double y2 = 276.8;
            return velocityFunctionY(distance);
        }else{
            return 0;
        }

    }
    @Override
    public double positionFunctionTheta(double s){
        return 0;
    }
    @Override
    public double velocityFunctionX(double s){
        if(autonomousStep == 1){
          return baseSpd;
        }else if(autonomousStep == 2){
            return baseSpd;
        } else{
            return 0;
        }
    }
    @Override
    public double velocityFunctionY(double s){
        if(autonomousStep == 1){
            return 0;
        } else if(autonomousStep ==2){
            return 0;
        } else{
            return 0;
        }
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
