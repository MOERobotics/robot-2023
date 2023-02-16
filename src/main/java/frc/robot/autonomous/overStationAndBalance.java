//THE FALLBACK AUTO - AUTO1

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class overStationAndBalance extends genericAutonomous{

    private final Timer m_timer = new Timer();
    double firstDist = 120;
    double secondDist = 60;
    double radius = 30;
    double desiredTheta = 5*Math.PI/2;
    double desiredInchesPerSecond = 12;
    double ds = desiredInchesPerSecond;
    double xspd, yspd, turnspd;
    double t;
    double velCalcT;
    double t1;

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
    double kP = 0.01;
    PIDController PID = new PIDController(kP,0,0);
    @Override
    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        autonomousStep = 0;
        robot.resetAttitude();
        robot.resetPIDPivot();
        robot.resetStartHeading();
        robot.resetStartDists();
        robot.resetStartPivots();
        PID.enableContinuousInput(-180,180);
    }
    @Override
    public void autonomousPeriodic(GenericRobot robot){
        baseSpd = 40.0;
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
                autonomousStep ++;
                break;
            case 1:
                t = m_timer.get();
                xspd = baseSpd;
                yspd = 0;
                autonomousStep++;
                break;
            case 2:
                t1 = m_timer.get();
                if(t1-t >= 7.5){
                    xspd = yspd = turnspd = 0;
                    autonomousStep++;
                }
                break;
            case 3:
                xspd = basePower;
                if (Math.abs(currPitch) > 5) {
                    autonomousStep++;
                }
                break;
            case 4:
//                        curPosOnRamp = base * Math.sin(currPitch) * (Math.cos(angleOfBoard) / Math.sin(angleOfBoard));
//                        leftside = basePower*(base - curPosOnRamp)/base;
//                        rightside = basePower*(base - curPosOnRamp)/base;
                xspd = basePower;
                if (Math.abs(currPitch) > 11) {
                    autonomousStep += 1;
                }
                break;
            case 5:
                xspd = climbPower;
                if (Math.abs(currPitch) < 10) {
                    autonomousStep++;
                }
                break;
            case 6:
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
            case 7:
                if (currPitch < -desiredPitch) {
                    xspd = -correctionPower;
                } else if (currPitch > desiredPitch) {
                    xspd = correctionPower;
                } else {
                    xspd = 0;
                    autonomousStep++;
                }
                break;
            case 8:
                if (Math.abs(currPitch) > desiredPitch) {
                    autonomousStep--;
                } else {
                    xspd = yspd = turnspd = 0;
                }
                break;
        }
        turnspd = PID.calculate(-robot.getYaw());
        robot.setDrive(xspd, yspd, turnspd);
    }


    @Override
    public double positionFunctionX(double s){
        return 0;
    }

    @Override
    public double positionFunctionY(double s){
        return 0;
    }
    @Override
    public double positionFunctionTheta(double s){
        return 0;
    }
    @Override
    public double velocityFunctionX(double s){
        if(s<=firstDist){
            return 1*ds;
        } else{
            return 0;
        }
    }
    @Override
    public double velocityFunctionY(double s){
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