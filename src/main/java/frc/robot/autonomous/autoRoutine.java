package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import org.opencv.core.Point;
import frc.robot.helpers.AutoCodeLines;
import edu.wpi.first.math.geometry.Rotation2d;


import static frc.robot.Robot.driveCode;

public class autoRoutine extends genericAutonomous {
    double xspd, yspd, turnspd;
    double radius = 30;
    double desiredTheta = 5 * Math.PI / 2;
    double desiredInchesPerSecond = 40;
    double ds = desiredInchesPerSecond;
    int autoStep;
  /*  double firstDist = 208.11;
    double secondDist = 220.08;
    double thirdDist = 220.75;
    double fourthDist = 221.76;
    double fifthDist = 63.43;*/
    double baseSpd = 25.0;
    double xPidK =1.0e-3;

    double yPidK = 1.0e-3; //1.0e-3;
    Point startPosition = new Point(55.88, 200.47);
    Point secondPosition = new Point(275.88,200.47);
    Point thirdPosition = new Point(275.88, 182.235);
    Point fourthPosition = new Point(55.88, 200.47);
    Point fifthPosition = new Point(55.8, 154.37);
    Point endPosition = new Point(115.17, 131.81);
    double kP = 0.5e-1;
    double s_0 = 0;
    PIDController PID = new PIDController(kP, 0, 0);

    double firstDist = AutoCodeLines.getDistance(startPosition,secondPosition) - 15;
    double secondDist = AutoCodeLines.getDistance(secondPosition, thirdPosition);
    double thirdDist = AutoCodeLines.getDistance(thirdPosition, fourthPosition);
    double fourthDist = AutoCodeLines.getDistance(fourthPosition, fifthPosition);

    double fifthDist = AutoCodeLines.getDistance(fifthPosition, endPosition);

    private final Timer m_timer = new Timer();

    double lengthOfField = 650.7;

    public void autonomousInit(GenericRobot robot) {
        m_timer.reset();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetStartHeading();
        autoStep = 0;
        if(robot.getRed()){
            yspd = -yspd;
            robot.setPose(new Pose2d(lengthOfField - startPosition.x, startPosition.y, new Rotation2d(0)));
        } else{
            robot.setPose(new Pose2d(startPosition.x, startPosition.y, new Rotation2d(0)));
        }
        robot.setPose(new Pose2d(startPosition.x, startPosition.y, new Rotation2d(0)));
        autoStep = 0;
        PID.enableContinuousInput(-180,180);
    }

    public void teleopPeriodic(GenericRobot robot) {
        driveCode.teleopPeriodic(robot);
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        double currPitch = robot.getRoll();
        double correctionPower = 14.0;
        double climbPower = 30.0;
        double basePower = 35.0;
        double desiredPitch = 9.0;

        SmartDashboard.putNumber("s", s_0);
        SmartDashboard.putNumber("autostep", autoStep);
        SmartDashboard.putNumber("xpsd", xspd);
        SmartDashboard.putNumber("yspd", yspd);
        SmartDashboard.putNumber("turn speed", turnspd);
        SmartDashboard.putNumber("first distance", firstDist);
        Pose2d currPose = robot.getPose();
        switch (autoStep) {
            case 0:
                m_timer.reset();
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                autoStep++;
            case 1:
                double t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0) + xPidK * (positionFunctionX(s_0) - currPose.getX());
                yspd = velocityFunctionY(s_0) - yPidK * (positionFunctionY(s_0) - currPose.getY());
                if (s_0 >= firstDist) {
                    xspd = 0;
                    yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }
                break;
            case 2:
                s_0 = getS(m_timer.get());
                xspd = velocityFunctionX(s_0)+ xPidK * (positionFunctionX(s_0) - currPose.getX());
                yspd = velocityFunctionY(s_0);// - yPidK * (positionFunctionY(s_0) - currPose.getY());

                if (s_0 >= secondDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }
                break;
            case 3:
                s_0 = getS(m_timer.get());
                xspd = velocityFunctionX(s_0) + xPidK * (positionFunctionX(s_0) - currPose.getX()) ;
                yspd = velocityFunctionY(s_0);// - yPidK * (positionFunctionY(s_0) - currPose.getY());
                if (s_0 >= thirdDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }
                break;
            case 4:
                s_0 = getS(m_timer.get());
                xspd = velocityFunctionX(s_0) + xPidK * (positionFunctionX(s_0) - currPose.getX());
                yspd = velocityFunctionY(s_0);// - yPidK * (positionFunctionY(s_0) - currPose.getY());
                if (s_0 >= fourthDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }

                break;
            case 5://right of chariging board
                s_0 = getS(m_timer.get());
                xspd = velocityFunctionX(s_0) + xPidK * (positionFunctionX(s_0) - currPose.getX());
                yspd = velocityFunctionY(s_0);// - yPidK * (positionFunctionY(s_0) - currPose.getY());
                if (s_0 >= fifthDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.stop();
                    autoStep++;
                }
                break;
     /*       case 5: //balance
                xspd = basePower;
                if (Math.abs(currPitch) > 5) {
                    autoStep++;
                }
                break;
            case 6:
//                        curPosOnRamp = base * Math.sin(currPitch) * (Math.cos(angleOfBoard) / Math.sin(angleOfBoard));
//                        leftside = basePower*(base - curPosOnRamp)/base;
//                        rightside = basePower*(base - curPosOnRamp)/base;
                xspd = basePower;
                if (Math.abs(currPitch) > 11) {
                    autoStep += 1;
                }
                break;
            case 7:
                xspd = climbPower;
                if (Math.abs(currPitch) < 10) {
                    autoStep++;
                }
                break;
            case 8:
                xspd = -correctionPower;
                autoStep++;
                break;
            case 9:
                if (currPitch < -desiredPitch) {
                    xspd = -correctionPower;
                } else if (currPitch > desiredPitch) {
                    xspd = correctionPower;
                } else {
                    xspd = 0;
                    autoStep++;
                }
                break;
            case 10:
                if (Math.abs(currPitch) > desiredPitch) {
                    autoStep--;
                } else {
                    xspd = yspd = turnspd = 0;
                }
                break; */


        }
        turnspd = PID.calculate(-robot.getYaw());
        robot.setDrive(xspd, yspd, turnspd);
    }

    public double positionFunctionX(double s) {
        if (autoStep == 0) {
            return startPosition.x * ds;
        }
        if (autoStep == 1) {
            return AutoCodeLines.getPositionX(startPosition, secondPosition, s)* ds;
        }
        if (autoStep == 2) {
            return AutoCodeLines.getPositionX(secondPosition, thirdPosition, s)* ds;
        }
        if (autoStep == 3) {
            return AutoCodeLines.getPositionX(thirdPosition, fourthPosition, s)* ds;
        }
        if (autoStep == 4) {
            return AutoCodeLines.getPositionX(fourthPosition, fifthPosition, s)* ds;
        }
        if (autoStep == 5) {
            return AutoCodeLines.getPositionX(fifthPosition, endPosition, s)* ds;
        }
        return endPosition.x;
    }

    @Override
    public double positionFunctionY(double s) {
        if (autoStep == 0) {
            return startPosition.y;
        }
        if (autoStep == 1) {
            return AutoCodeLines.getPositionY(startPosition, secondPosition, s)* ds;
        }
        if (autoStep == 2) {
            return AutoCodeLines.getPositionY(secondPosition, thirdPosition, s)* ds;
        }
        if (autoStep == 3) {
            return AutoCodeLines.getPositionY(thirdPosition, fourthPosition, s)* ds;
        }
        if (autoStep == 4) {
            return AutoCodeLines.getPositionY(fourthPosition, fifthPosition, s)* ds;
        }
        if (autoStep == 5) {
            return AutoCodeLines.getPositionY(fifthPosition, endPosition, s)* ds;
        }
        return endPosition.y;
    }

    public double velocityFunctionX(double s) {
        if (autoStep == 0){
            return 0;
        }
        if (autoStep == 1){
            return AutoCodeLines.getVelocityX(startPosition, secondPosition, s) * ds;
        }
        if (autoStep == 2){
            return AutoCodeLines.getVelocityX(secondPosition, thirdPosition, s)* ds;
        }
        if (autoStep == 3){
            return AutoCodeLines.getVelocityX(thirdPosition, fourthPosition, s)* ds;
        }
        if (autoStep == 4){
            return AutoCodeLines.getVelocityX(fourthPosition, fifthPosition, s)* ds;
        }
        if (autoStep == 5){
            return AutoCodeLines.getVelocityX(fifthPosition, endPosition, s)* ds;
        }
        return 0;
    }



    @Override
    public double velocityFunctionY(double s) {
        if (autoStep == 0){
            return 0;
        }
        if (autoStep == 1){
            return AutoCodeLines.getVelocityY(startPosition, secondPosition, s)* ds;
        }
        if (autoStep == 2){
            return AutoCodeLines.getVelocityY(secondPosition, thirdPosition, s)* ds;
        }
        if (autoStep == 3){
            return AutoCodeLines.getVelocityY(thirdPosition, fourthPosition, s)* ds;
        }
        if (autoStep == 4){
            return AutoCodeLines.getVelocityY(fourthPosition, fifthPosition, s)* ds;
        }
        if (autoStep == 5){
            return AutoCodeLines.getVelocityY(fifthPosition, endPosition, s)* ds;
        }
        return 0;
    }

    @Override
    public double getS(double time){
        return time*ds;
    }



}
