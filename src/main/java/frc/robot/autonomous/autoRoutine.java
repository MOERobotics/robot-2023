package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import org.opencv.core.Point;
import frc.robot.helpers.AutoCodeLines;
import edu.wpi.first.math.geometry.Rotation2d;

public class autoRoutine extends genericAutonomous {
    double xspd, yspd, turnspd;
    double desiredInchesPerSecond = 40;
    double ds = desiredInchesPerSecond;
    int autoStep;
    double xPidK =0;//1.0e-2;

    double yPidK = 0;//1.0e-2; //1.0e-3;
    Point startPosition = new Point(55.88, 200.47);
    //Point secondPosition = new Point(275.88,200.47);
    Point secondPosition = new Point(275.88, 182.235);
    Point thirdPosition = new Point(55.88, 200.47);
    Point fourthPosition = new Point(55.8, 154.37);
    Point endPosition = new Point(115.17, 131.81);
    double kP = 0.5e-1;
    double s = 0;
    PIDController PID = new PIDController(kP, 0, 0);

    double firstDist = AutoCodeLines.getDistance(startPosition,secondPosition);
    double secondDist = AutoCodeLines.getDistance(secondPosition, thirdPosition);
    double thirdDist = AutoCodeLines.getDistance(thirdPosition, fourthPosition);
    double fourthDist = AutoCodeLines.getDistance(fourthPosition, endPosition);

    private final Timer m_timer = new Timer();

    double lengthOfField = 650.7;

    public void autonomousInit(GenericRobot robot) {
        m_timer.reset();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetStartHeading();
        Rotation2d startRot = new Rotation2d(0);
        autoStep = 0;
        if(robot.getRed()){
            startPosition.x = lengthOfField - startPosition.x;
            secondPosition.x = lengthOfField - secondPosition.x;
            thirdPosition.x = lengthOfField - thirdPosition.x;
            fourthPosition.x = lengthOfField - fourthPosition.x;
            endPosition.x = lengthOfField - endPosition.x;
            startRot = new Rotation2d(Math.PI);
        }
        robot.setPose(new Pose2d(startPosition.x, startPosition.y, startRot));
        autoStep = 0;
        PID.enableContinuousInput(-180,180);
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        double currPitch = robot.getRoll();
        double correctionPower = 14.0;
        double climbPower = 30.0;
        double basePower = 35.0;
        double desiredPitch = 9.0;

        SmartDashboard.putNumber("s", s);
        SmartDashboard.putNumber("autostep", autoStep);
        SmartDashboard.putNumber("xpsd", xspd);
        SmartDashboard.putNumber("yspd", yspd);
        SmartDashboard.putNumber("turn speed", turnspd);
        SmartDashboard.putNumber("first distance", firstDist);
        Pose2d currPose = robot.getPose();

        switch (autoStep) {
            case 0: //resets everything
                m_timer.reset();
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                autoStep++;
            case 1:
                double t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= firstDist) {
                    xspd = 0;
                    yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }
                break;
            case 2:
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s) + yPidK * (positionFunctionY(s) - currPose.getY());

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
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s) + xPidK * (positionFunctionX(s) - currPose.getX()) ;
                yspd = velocityFunctionY(s) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= thirdDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }
                break;
            case 4:
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= fourthDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autoStep++;
                }
                break;
            /*case 5:
                xspd = basePower;
                if (Math.abs(currPitch) > 11) { // driving up charge station
                    autonomousStep ++;
                }
                break;
            case 6:
                xspd = climbPower;
                if (Math.abs(currPitch) < 10) { //flattened out
                    autonomousStep++;
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
*/

        }
        if (autoStep > 0) turnspd = PID.calculate(-robot.getYaw());
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
            return AutoCodeLines.getVelocityX(fourthPosition, endPosition, s)* ds;
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
            return AutoCodeLines.getVelocityY(fourthPosition, endPosition, s)* ds;
        }
        return 0;
    }

    @Override
    public double getS(double time){
        return time*ds;
    }



}
