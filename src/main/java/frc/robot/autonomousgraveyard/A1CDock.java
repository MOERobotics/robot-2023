package frc.robot.autonomousgraveyard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.genericAutonomous;
import frc.robot.generic.GenericRobot;
import frc.robot.vision.Detection;
import frc.robot.vision.MoeNetVision;
import org.opencv.core.Point;
import frc.robot.helpers.AutoCodeLines;
import edu.wpi.first.math.geometry.Rotation2d;

public class A1CDock extends genericAutonomous {
    double TARGET_DISTANCE = 0; //this is distance of camera
    double xspd, yspd, turnspd;
    double desiredInchesPerSecond = 70;
    double xPidK = 7;
    double yPidK = 7;

    double defaultSpeed = 40;
    boolean autoMode = false;
////////////////////////////////////////////////////////////////////////////////////////////////Point stuff
    Point startPositionBlue = new Point(55.88+4, 195.47);
    Point startPosition = new Point(startPositionBlue.x, startPositionBlue.y);
    Point secondPositionBlue = new Point(188.88, 192.735); //275.88, 186.235
    Point secondPosition = new Point(secondPositionBlue.x, secondPositionBlue.y);
    Point thirdPositionBlue = new Point(55.88+9, 195.47); //55.8,199.43
    Point thirdPosition = new Point(thirdPositionBlue.x, thirdPositionBlue.y);
    Point fourthPositionBlue = new Point(55.8+9, 148.37); //54.5,154.89
    Point fourthPosition = new Point(fourthPositionBlue.x, fourthPositionBlue.y);
    Point endPositionBlue = new Point(105.17-12, 120.81); //114.67,131.96
    Point endPosition = new Point(endPositionBlue.x, endPositionBlue.y);

    double firstDist = AutoCodeLines.getDistance(startPosition,secondPosition);
    double secondDist = AutoCodeLines.getDistance(secondPosition, thirdPosition);
    double thirdDist = AutoCodeLines.getDistance(thirdPosition, fourthPosition);
    double fourthDist = AutoCodeLines.getDistance(fourthPosition, endPosition);

    double centerLineBlue = 300;
    double centerLine = centerLineBlue;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double kP = 1.0e-1; //.5e-1

    double s = 0;
    PIDController PID = new PIDController(kP, 0, 0);
    private final Timer m_timer = new Timer();
    double lengthOfField = 650.7;
    double correctionPowerBlue = 20.0;
    double climbPowerBlue = 35.0;
    double basePowerBlue = 35.0;
    double correctionPower = correctionPowerBlue;
    double climbPower = climbPowerBlue;
    double basePower = basePowerBlue;
    boolean collectorUp = true;
    double collectorRPM = 0;
    boolean openGripper = true;
    double armPos = 0;
    Rotation2d startRot;
    MoeNetVision vision;
    Pose2d desiredPoseBlue = new Pose2d(275.88, 189.235, new Rotation2d(0)); //default pos
    Pose2d desiredPose = new Pose2d(desiredPoseBlue.getX(), desiredPoseBlue.getY(), desiredPoseBlue.getRotation());


    public void autonomousInit(GenericRobot robot) {
        vision = new MoeNetVision(robot);
        startRot = new Rotation2d(0);
        autonomousStep = 0;
        autoMode = false;
        robot.setPigeonYaw(0);
        if(robot.getRed()){
            startRot = new Rotation2d(Math.PI);
            startPosition.x = lengthOfField - startPositionBlue.x;
            secondPosition.x = lengthOfField - secondPositionBlue.x;
            thirdPosition.x = lengthOfField - thirdPositionBlue.x;
            fourthPosition.x = lengthOfField - fourthPositionBlue.x;
            endPosition.x = lengthOfField - endPositionBlue.x;
            desiredPose = new Pose2d(lengthOfField - desiredPoseBlue.getX(),
                    desiredPoseBlue.getY(), startRot);
            correctionPower = -correctionPowerBlue;
            climbPower = -climbPowerBlue;
            basePower = -basePowerBlue;
            centerLine = lengthOfField - centerLineBlue;
            robot.setPigeonYaw(180);
        }else{
            startPosition.x = startPositionBlue.x;
            secondPosition.x = secondPositionBlue.x;
            thirdPosition.x = thirdPositionBlue.x;
            fourthPosition.x = fourthPositionBlue.x;
            endPosition.x = endPositionBlue.x;
            correctionPower = correctionPowerBlue;
            desiredPose = new Pose2d(desiredPoseBlue.getX(),
                    desiredPoseBlue.getY(), startRot);
            climbPower = climbPowerBlue;
            basePower = basePowerBlue;
            centerLine = centerLineBlue;
        }
        robot.resetStartHeading();
        m_timer.reset();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetAttitude();
        robot.setPose(new Pose2d(startPosition.x, startPosition.y, startRot));
        autonomousStep = 0;
        PID.enableContinuousInput(-180,180);
        xspd = yspd = 0;
        openGripper = true;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        double currPitch = robot.getPitch();

        double desiredPitch = 9.0;
        Pose2d currPose = robot.getPose();
        SmartDashboard.putNumber("s", s);
        SmartDashboard.putNumber("autostep", autonomousStep);
        SmartDashboard.putNumber("xpsd", xspd);
        SmartDashboard.putNumber("yspd", yspd);
        SmartDashboard.putNumber("turn speed", turnspd);
        SmartDashboard.putNumber("first distance", firstDist);
        SmartDashboard.putBoolean("do I think im red?", robot.getRed());
        SmartDashboard.putNumber("startPosition", startPosition.x);
        SmartDashboard.putNumber("startBlue", startPositionBlue.x);
        SmartDashboard.putNumber("x correction", xPidK * (positionFunctionX(s) - currPose.getX()));
        SmartDashboard.putNumber("y correction", yPidK * (positionFunctionY(s) - currPose.getY()));

        switch (autonomousStep) {
            case 0: //resets everything
                armPos = 20;
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                collectorRPM = 9000;
                collectorUp = false;
                openGripper = true;
                autoMode = true;
                robot.setPose(new Pose2d(startPosition.x, startPosition.y, startRot));
                xspd = yspd = turnspd = 0;
                if (m_timer.get() > .5){
                    autonomousStep++;
                    autoMode = false;
                    m_timer.reset();
                    m_timer.get();
                }
                break;
            case 1:
                armPos = -4;
                autoMode = false;
                double t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoDetected()) collectorRPM = 4000;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                Detection firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d();
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);
                    Pose2d possPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    if (possPose.getX() > centerLine && robot.getRed()){
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    }
                    if (possPose.getX() < centerLine && !robot.getRed()){
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    }
                    SmartDashboard.putString("detautoTarget", String.format("%f, %f", this.desiredPose.getX(), this.desiredPose.getY()));
                }
/////////////////////////////////////////////////////////////////////////////////vision detection code
                if (s >= firstDist) {
                    xspd = 0;
                    yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep += 1;
                }
                break;
            case 2: ///object detection step
                firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d();
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);
                    Pose2d possPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    if (possPose.getX() > centerLine && robot.getRed()){
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    }
                    if (possPose.getX() < centerLine && !robot.getRed()){
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    }
                    SmartDashboard.putString("detautoTarget", String.format("%f, %f", this.desiredPose.getX(), this.desiredPose.getY()));
                }

                double xDiff = desiredPose.getX()-currPose.getX() + 6;
                double yDiff = desiredPose.getY()-currPose.getY();
                double totDiff = Math.hypot(xDiff, yDiff);
                xspd = yspd = 0;
                if (totDiff > 0) xspd = defaultSpeed * xDiff/totDiff;
                if (totDiff > 0) yspd = defaultSpeed * yDiff/totDiff;
                if (robot.cargoDetected() || m_timer.get() > 6){
                    secondPosition = new Point(currPose.getX(), currPose.getY());
                    secondDist = AutoCodeLines.getDistance(secondPosition, thirdPosition);
                    collectorRPM = 4000;
                    xspd = yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep += 1;
                }
                break;

            case 3:
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoDetected()) collectorRPM = 4000;
                if (s >= secondDist) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 4:
                collectorUp = true;
                collectorRPM = 0;
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
                    autonomousStep += 1;
                    autoMode = true;
                    collectorRPM = 9000;
                }
                break;
            case 5:
                xspd = yspd = turnspd = 0;
                autoMode = true;
                armPos = 45;
                collectorRPM = 9000;
                if (m_timer.get() >= 1){
                    autonomousStep ++;
                    m_timer.reset();
                    m_timer.start();
                }
                break;
            case 6:
                collectorRPM = 0;
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= fourthDist) {
                    armPos = -4;
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 7:
                xspd = basePower;
                if (Math.abs(currPitch) > 11) { // driving toward and up charge station until fully pitched up.
                    autonomousStep++;
                }
                break;
            case 8:
                xspd = climbPower;
                collectorRPM = 0;
                if (Math.abs(currPitch) < 10) { //forward until flattened out
                    autonomousStep++;
                }
                break;
            case 9:
                if (currPitch < -desiredPitch) { //correcting begins
                    xspd = -correctionPower; //backward
                } else if (currPitch > desiredPitch) {
                    xspd = correctionPower; //forward
                } else {
                    xspd = 0;
                }
                break;
            case 10:
                xspd = yspd = 0;
                break;

        }
        if ((autonomousStep > 0 && autonomousStep < 7)) turnspd = PID.calculate(-robot.getYaw());
        robot.raiseTopRoller(collectorUp);
        robot.setDrive(xspd, yspd, turnspd, true, true);
        robot.collect(collectorRPM, autoMode);
        robot.openGripper(openGripper);
        robot.holdArmPosition(armPos);


    }

    public double positionFunctionX(double s) {
        if (autonomousStep == 0) {
            return startPosition.x;
        }
        if (autonomousStep == 1) {
            return AutoCodeLines.getPositionX(startPosition, secondPosition, s);
        }
        if (autonomousStep == 3) {
            return AutoCodeLines.getPositionX(secondPosition, thirdPosition, s);
        }
        if (autonomousStep == 4) {
            return AutoCodeLines.getPositionX(thirdPosition, fourthPosition, s);
        }
        if (autonomousStep == 6) {
            return AutoCodeLines.getPositionX(fourthPosition, endPosition, s);
        }
        return endPosition.x;
    }

    @Override
    public double positionFunctionY(double s) {
        if (autonomousStep == 0) {
            return startPosition.y;
        }
        if (autonomousStep == 1) {
            return AutoCodeLines.getPositionY(startPosition, secondPosition, s);
        }
        if (autonomousStep == 3) {
            return AutoCodeLines.getPositionY(secondPosition, thirdPosition, s);
        }
        if (autonomousStep == 4) {
            return AutoCodeLines.getPositionY(thirdPosition, fourthPosition, s);
        }
        if (autonomousStep == 6) {
            return AutoCodeLines.getPositionY(fourthPosition, endPosition, s);
        }
        return endPosition.y;
    }

    public double velocityFunctionX(double s, double time) {
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityX(startPosition, secondPosition, s) * getdS(time);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getVelocityX(secondPosition, thirdPosition, s)* getdS(time);
        }
        if (autonomousStep == 4){
            return AutoCodeLines.getVelocityX(thirdPosition, fourthPosition, s)* getdS(time);
        }
        if (autonomousStep == 6){
            return AutoCodeLines.getVelocityX(fourthPosition, endPosition, s)* getdS(time);
        }
        return 0;
    }



    @Override
    public double velocityFunctionY(double s, double time) {
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityY(startPosition, secondPosition, s)* getdS(time);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getVelocityY(secondPosition, thirdPosition, s)* getdS(time);
        }
        if (autonomousStep == 4){
            return AutoCodeLines.getVelocityY(thirdPosition, fourthPosition, s)* getdS(time);
        }
        if (autonomousStep == 6){
            return AutoCodeLines.getVelocityY(fourthPosition, endPosition, s)* getdS(time);
        }
        return 0;
    }

    @Override
    public double getS(double time){
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getS(firstDist, .55, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getS(secondDist, .55, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 4){
            return AutoCodeLines.getS(thirdDist, .25, desiredInchesPerSecond-20, time);
        }
        if (autonomousStep == 6){
            return AutoCodeLines.getS(fourthDist, .25, desiredInchesPerSecond-20, time);
        }
        return 0;
    }
    @Override
    public double getdS(double time){
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getdS(firstDist, .55, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getdS(secondDist, .55, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 4){
            return AutoCodeLines.getdS(thirdDist, .25, desiredInchesPerSecond-20, time);
        }
        if (autonomousStep == 6){
            return AutoCodeLines.getdS(fourthDist, .25, desiredInchesPerSecond-20, time);
        }
        return 0;
    }



}
