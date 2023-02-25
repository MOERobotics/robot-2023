package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import org.opencv.core.Point;
import frc.robot.helpers.AutoCodeLines;
import edu.wpi.first.math.geometry.Rotation2d;

public class A1CDock extends genericAutonomous {
    double xspd, yspd, turnspd;
    double desiredInchesPerSecond = 70;
    double ds = desiredInchesPerSecond;
    int autoStep;
    double xPidK = 7;

    double widthRobot = 34;

    double yPidK = 7;
    Point startPositionBlue = new Point(55.88+4, 200.47);
    Point startPosition = new Point(startPositionBlue.x, startPositionBlue.y);
    //Point secondPosition = new Point(275.88,200.47);
    Point secondPositionBlue = new Point(275.88-20, 182.235); //269.3,180.8
    Point secondPosition = new Point(secondPositionBlue.x, secondPositionBlue.y);
    Point thirdPositionBlue = new Point(55.88+10, 200.47); //55.8,199.43
    Point thirdPosition = new Point(thirdPositionBlue.x, thirdPositionBlue.y);
    Point fourthPositionBlue = new Point(55.8+10, 154.37); //54.5,154.89
    Point fourthPosition = new Point(fourthPositionBlue.x, fourthPositionBlue.y);
    Point endPositionBlue = new Point(105.17-12, 120.81); //114.67,131.96
    Point endPosition = new Point(endPositionBlue.x, endPositionBlue.y);
    double kP = 0.7e-1; //.5e-1

    double s = 0;
    PIDController PID = new PIDController(kP, 0, 0);

    double firstDist = AutoCodeLines.getDistance(startPosition,secondPosition);
    double secondDist = AutoCodeLines.getDistance(secondPosition, thirdPosition);
    double thirdDist = AutoCodeLines.getDistance(thirdPosition, fourthPosition);
    double fourthDist = AutoCodeLines.getDistance(fourthPosition, endPosition);

    private final Timer m_timer = new Timer();

    double lengthOfField = 650.7;

    double correctionPowerBlue = 18.0;
    double climbPowerBlue = 30.0;
    double basePowerBlue = 35.0;

    double correctionPower = correctionPowerBlue;
    double climbPower = climbPowerBlue;
    double basePower = basePowerBlue;
    Rotation2d startRot;

    public void autonomousInit(GenericRobot robot) {
        startRot = new Rotation2d(0);
        autoStep = 0;
        robot.setPigeonYaw(0);
        if(robot.getRed()){
            startPosition.x = lengthOfField - startPositionBlue.x;
            secondPosition.x = lengthOfField - secondPositionBlue.x;
            thirdPosition.x = lengthOfField - thirdPositionBlue.x;
            fourthPosition.x = lengthOfField - fourthPositionBlue.x;
            endPosition.x = lengthOfField - endPositionBlue.x;
            correctionPower = -correctionPowerBlue;
            climbPower = -climbPowerBlue;
            basePower = -basePowerBlue;
            startRot = new Rotation2d(Math.PI);
            robot.setPigeonYaw(180);
        }else{
            startPosition.x = startPositionBlue.x;
            secondPosition.x = secondPositionBlue.x;
            thirdPosition.x = thirdPositionBlue.x;
            fourthPosition.x = fourthPositionBlue.x;
            endPosition.x = endPositionBlue.x;
            correctionPower = correctionPowerBlue;
            climbPower = climbPowerBlue;
            basePower = basePowerBlue;
        }
        robot.resetStartHeading();
        m_timer.reset();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetAttitude();
        robot.setPose(new Pose2d(startPosition.x, startPosition.y, startRot));
        autoStep = 0;
        PID.enableContinuousInput(-180,180);
        xspd = yspd = 0;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        double currPitch = robot.getPitch();

        double desiredPitch = 9.0;
        Pose2d currPose = robot.getPose();
        SmartDashboard.putNumber("s", s);
        SmartDashboard.putNumber("autostep", autoStep);
        SmartDashboard.putNumber("xpsd", xspd);
        SmartDashboard.putNumber("yspd", yspd);
        SmartDashboard.putNumber("turn speed", turnspd);
        SmartDashboard.putNumber("first distance", firstDist);
        SmartDashboard.putBoolean("do I think im red?", robot.getRed());
        SmartDashboard.putNumber("startPosition", startPosition.x);
        SmartDashboard.putNumber("startBlue", startPositionBlue.x);
        SmartDashboard.putNumber("x correction", xPidK * (positionFunctionX(s) - currPose.getX()));
        SmartDashboard.putNumber("y correction", yPidK * (positionFunctionY(s) - currPose.getY()));

        switch (autoStep) {
            case 0: //resets everything
                m_timer.reset();
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                robot.setPose(new Pose2d(startPosition.x, startPosition.y, startRot));
                xspd = yspd = turnspd = 0;
                autoStep ++;
                break;
            case 1:
                double t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= firstDist) {
                    xspd = 0;
                    yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }
                break;
            case 2:
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());

                if (s >= secondDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }
                break;
            case 3:
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX()) ;
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= thirdDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep = 15;
                }
                break;
            case 15:
                xspd = yspd = turnspd = 0;
                if (m_timer.get() >= 1){
                    autoStep = 4;
                    m_timer.reset();
                    m_timer.start();
                }
                break;
            case 4:
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= fourthDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }
                break;
            case 5:
                xspd = basePower;
                if (Math.abs(currPitch) > 11) { // driving up charge station
                    autoStep ++;
                }
                break;
            case 6:
                xspd = climbPower;
                if (Math.abs(currPitch) < 10) { //flattened out
                    autoStep++;
                }
                break;
            case 7:
                if (currPitch < -desiredPitch) { //correcting begins
                    xspd = -correctionPower; //backward
                } else if (currPitch > desiredPitch) {
                    xspd = correctionPower; //forward
                } else {
                    xspd = 0;
                }
                break;

        }
        if (autoStep > 0 && autoStep < 5) turnspd = PID.calculate(-robot.getYaw());
        robot.setDrive(xspd, yspd, turnspd, true);
    }

    public double positionFunctionX(double s) {
        if (autoStep == 0) {
            return startPosition.x;
        }
        if (autoStep == 1) {
            return AutoCodeLines.getPositionX(startPosition, secondPosition, s);
        }
        if (autoStep == 2) {
            return AutoCodeLines.getPositionX(secondPosition, thirdPosition, s);
        }
        if (autoStep == 3) {
            return AutoCodeLines.getPositionX(thirdPosition, fourthPosition, s);
        }
        if (autoStep == 4) {
            return AutoCodeLines.getPositionX(fourthPosition, endPosition, s);
        }
        return endPosition.x;
    }

    @Override
    public double positionFunctionY(double s) {
        if (autoStep == 0) {
            return startPosition.y;
        }
        if (autoStep == 1) {
            return AutoCodeLines.getPositionY(startPosition, secondPosition, s);
        }
        if (autoStep == 2) {
            return AutoCodeLines.getPositionY(secondPosition, thirdPosition, s);
        }
        if (autoStep == 3) {
            return AutoCodeLines.getPositionY(thirdPosition, fourthPosition, s);
        }
        if (autoStep == 4) {
            return AutoCodeLines.getPositionY(fourthPosition, endPosition, s);
        }
        return endPosition.y;
    }

    public double velocityFunctionX(double s, double time) {
        if (autoStep == 0){
            return 0;
        }
        if (autoStep == 1){
            return AutoCodeLines.getVelocityX(startPosition, secondPosition, s) * getdS(time);
        }
        if (autoStep == 2){
            return AutoCodeLines.getVelocityX(secondPosition, thirdPosition, s)* getdS(time);
        }
        if (autoStep == 3){
            return AutoCodeLines.getVelocityX(thirdPosition, fourthPosition, s)* getdS(time);
        }
        if (autoStep == 4){
            return AutoCodeLines.getVelocityX(fourthPosition, endPosition, s)* getdS(time);
        }
        return 0;
    }



    @Override
    public double velocityFunctionY(double s, double time) {
        if (autoStep == 0){
            return 0;
        }
        if (autoStep == 1){
            return AutoCodeLines.getVelocityY(startPosition, secondPosition, s)* getdS(time);
        }
        if (autoStep == 2){
            return AutoCodeLines.getVelocityY(secondPosition, thirdPosition, s)* getdS(time);
        }
        if (autoStep == 3){
            return AutoCodeLines.getVelocityY(thirdPosition, fourthPosition, s)* getdS(time);
        }
        if (autoStep == 4){
            return AutoCodeLines.getVelocityY(fourthPosition, endPosition, s)* getdS(time);
        }
        return 0;
    }

    @Override
    public double getS(double time){
        if (autoStep == 0){
            return 0;
        }
        if (autoStep == 1){
            return AutoCodeLines.getS(firstDist, .5, desiredInchesPerSecond, time);
        }
        if (autoStep == 2){
            return AutoCodeLines.getS(secondDist, .5, desiredInchesPerSecond, time);
        }
        if (autoStep == 3){
            return AutoCodeLines.getS(thirdDist, .2, desiredInchesPerSecond-30, time);
        }
        if (autoStep == 4){
            return AutoCodeLines.getS(fourthDist, .2, desiredInchesPerSecond-30, time);
        }
        return 0;
    }
    @Override
    public double getdS(double time){
        if (autoStep == 0){
            return 0;
        }
        if (autoStep == 1){
            return AutoCodeLines.getdS(firstDist, .5, desiredInchesPerSecond, time);
        }
        if (autoStep == 2){
            return AutoCodeLines.getdS(secondDist, .5, desiredInchesPerSecond, time);
        }
        if (autoStep == 3){
            return AutoCodeLines.getdS(thirdDist, .2, desiredInchesPerSecond-30, time);
        }
        if (autoStep == 4){
            return AutoCodeLines.getdS(fourthDist, .2, desiredInchesPerSecond-30, time);
        }
        return 0;
    }



}
