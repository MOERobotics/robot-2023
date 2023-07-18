package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import frc.robot.vision.Detection;
import frc.robot.vision.MoeNetVision;
import org.opencv.core.Point;

public class FEngage extends genericAutonomous{

    Point startPosition       = new Point(59,108);
    Point firstScorePosition  = new Point(69, 108);
    double xLeftChargeStation = 230;
    Point estimatedCubeSpot   = new Point(246, 112);
    Point spotBeforeEngage    = new Point(220, 108);
    Point startPositionBlue       = new Point(85,108);
    Point firstScorePositionBlue  = new Point(69, 108);
    double xLeftChargeStationBlue     = 230;
    Point estimatedCubeSpotBlue   = new Point(240, 112);
    Point spotBeforeEngageBlue    = new Point(220, 108);

    double dist1 = AutoCodeLines.getDistance(startPositionBlue, firstScorePositionBlue);
    double dist2 = AutoCodeLines.getDistance(estimatedCubeSpotBlue, spotBeforeEngageBlue);
    Pose2d desiredCubePos;
    double lengthOfField = 650;

    MoeNetVision vision;

    Rotation2d startRot;
    boolean lightOn, collectorUp, openGripper;
    Timer m_timer = new Timer();
    double armPos = 0;
    double xspd, yspd, turnspd, s, t, xPos, timerDelta;
    double basePower = 60;
    double currPitch;
    double climbPower = 60;
    double correctionPower = 13;
    double defaultSpeed = 40;
    double desiredPitch = 9;
    double dropping = 9;
    double high = 13.5;
    double firstBreak = 5;
    double desiredInchesPerSecond = 60;
    double xPidK = 7;
    double yPidK = 7;
    double collectorRPM = 9000;
    Pose3d visionPose;
    PIDController PID = new PIDController(0.7e-1, 0, 0);
    double startXPose;
    double centerLineBlue = 310;
    double centerLine = centerLineBlue;
    @Override
    public void autonomousInit(GenericRobot robot) {
        PID.enableContinuousInput(-180,180);
        xspd = yspd = turnspd = 0;
        vision = new MoeNetVision(robot);
        startRot = new Rotation2d(0);
        autonomousStep = -1;
        lightOn = false;
        robot.setPigeonYaw(0);
        startPosition.x        = startPositionBlue.x;
        firstScorePosition.x   = firstScorePositionBlue.x;
        estimatedCubeSpot.x    = estimatedCubeSpotBlue.x;
        spotBeforeEngage.x     = spotBeforeEngageBlue.x;
        basePower = 60;
        climbPower = 60;
        correctionPower = 13;
        xLeftChargeStation = xLeftChargeStationBlue;
        centerLine = centerLineBlue;
        if (robot.getRed()){
            centerLine = lengthOfField - centerLineBlue;
            startPosition.x        = lengthOfField - startPositionBlue.x;
            firstScorePosition.x   = lengthOfField - firstScorePositionBlue.x;
            estimatedCubeSpot.x    = lengthOfField - estimatedCubeSpotBlue.x;
            spotBeforeEngage.x     = lengthOfField - spotBeforeEngageBlue.x;
            xLeftChargeStation     = lengthOfField - xLeftChargeStationBlue;
            basePower *= -1;
            climbPower *= -1;
            correctionPower *= -1;
            startRot = new Rotation2d(Math.PI);
            robot.setPigeonYaw(180);
        }
        desiredCubePos = new Pose2d(estimatedCubeSpot.x, estimatedCubeSpot.y, startRot);
        robot.resetPose();
        robot.resetAttitude();
        robot.setPose(new Pose2d(startPosition.x, startPosition.y, startRot));
        m_timer.restart();
        openGripper = false;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("autoStep", autonomousStep);
        Pose2d currPose = robot.getPose();
        currPitch = robot.getPitch();
        visionPose = vision.getPose();
        switch(autonomousStep){
            case -1:
                if (m_timer.get() > .1){
                    autonomousStep ++;
                }
                break;
            case 0: //resets everything
                collectorUp = false;
                openGripper = false;
                collectorRPM = 0;
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                autonomousStep ++;
                m_timer.reset();
                m_timer.get();
                startXPose = currPose.getX();
                break;
            case 1: //roll forward and lift your arm
                xspd = 40;
                if (robot.getRed()) xspd *= -1;
                yspd = 0;
                if(Math.abs(currPose.getX() - startXPose) >= 24){
                    xspd = 0;
                    armPos = 97;
                    if (Math.abs(robot.getPotDegrees() - armPos) <= 10){
                        startXPose = currPose.getX();
                        m_timer.restart();
                        autonomousStep ++;
                    }
                }
                break;
            case 2: //rollback to get ready to score
                xspd = -40;
                if (robot.getRed()) xspd *= -1;
                if (Math.abs(startXPose - currPose.getX()) >= 24){
                    xspd = yspd = 0;
                    m_timer.restart();
                    autonomousStep++;
                }
                break;
            case 3: //score the cone
                armPos = 81;
                if (m_timer.get() > .2){
                    openGripper = true;
                    if (m_timer.get() > .4) {
                        m_timer.restart();
                        autonomousStep++;
                        startXPose = currPose.getX();
                    }
                }
                break;
            case 4: //drive over the charge station
                xspd = basePower;
                yspd = turnspd = 0;
                if (Math.abs(currPose.getX()-startXPose) >= 20){
                    armPos = -4;
                    xspd = 0;
                    m_timer.restart();
                    autonomousStep ++;
                }
                break;
            case 5:
                if (robot.getPotDegrees()<5){
                    xspd = basePower;
                    if (Math.abs(currPitch) > firstBreak) {
                        armPos = -4;
                        m_timer.restart();
                        //Add length of the robot from front encoder to end of back wheel.
                        autonomousStep ++;
                    }
                }
                break;
            case 6: //are we on the incline
                xspd = basePower;
                if (currPitch > high) {
                    xPos = robot.getPose().getX();
                    autonomousStep++;
                }
                break;
            case 7: //flattened
                xspd = climbPower;
                if (Math.abs(currPitch) < dropping) {
                    autonomousStep++;
                }
                break;
            case 8: //down incline
                xspd = climbPower+4;
                if (Math.abs(currPitch) > high){

                    autonomousStep ++;
                }
                break;
            case 9://on ground
                xspd = climbPower+4;
                if (Math.abs(currPitch) < desiredPitch){
                    collectorUp = false;
                    collectorRPM = 0;
                    robot.resetPose();
                    robot.setPose(new Pose2d(xLeftChargeStation, currPose.getY(), startRot));
                    m_timer.restart();
                    autonomousStep ++;
                }
                break;
            case 10: //get cube

                double xDiff = desiredCubePos.getX()-currPose.getX() + 6;
                double yDiff = desiredCubePos.getY()-currPose.getY();
                double totDiff = Math.hypot(xDiff, yDiff);
                xspd = yspd = 0;
                if (totDiff > 0) xspd = defaultSpeed * xDiff/totDiff;
                if (totDiff > 0) yspd = defaultSpeed * yDiff/totDiff;
                if (robot.cargoDetected() || m_timer.get() > 3 || totDiff < 2){
                    estimatedCubeSpot = new Point(currPose.getX(), currPose.getY());
                    dist2 = AutoCodeLines.getDistance(estimatedCubeSpot, spotBeforeEngage);
                    collectorRPM = 0;
                    xspd = yspd = 0;
                    m_timer.restart();
                    climbPower*= -1;
                    basePower*= -1;
                    autonomousStep += 1;
                }
                break;
            case 11: //go to spot before engage
                SmartDashboard.putNumber("dist2", dist2);
                collectorRPM = 0;
                t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s, t) + xPidK*(positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK*(positionFunctionY(s) - currPose.getY());
                if (s >= dist2){
                    xspd = yspd = 0;
                    m_timer.restart();
                    autonomousStep++;
                }
                break;
            case 12:
                xspd = basePower;
                if (Math.abs(currPitch) > high) {
                    xPos = robot.getPose().getX();
                    autonomousStep++;
                }
                break;
            case 13: // go up incline
                if(Math.abs(robot.getPose().getX() - xPos) >= 34+25){
                    climbPower = Math.signum(climbPower)*13;
                }
                xspd = climbPower;
                if (Math.abs(currPitch) < dropping) {
                    xspd = 0;
                    autonomousStep++;
                }
                break;
            case 14: // start
                xspd = correctionPower;
                m_timer.reset();
                m_timer.start();
                //This is a future feature to stop and let others get on before autobalancing.
                autonomousStep++;
                break;
            case 15:
                if ((currPitch < -desiredPitch) && (m_timer.get() <1)) { //correcting begins
                    timerDelta = m_timer.get();
                    xspd = -correctionPower; //backward
                } else if ((currPitch > desiredPitch) && (m_timer.get() <1)) {
                    timerDelta = m_timer.get();
                    xspd = correctionPower;//forward
                } else {
                    xspd = 0;
                    if (m_timer.get()-timerDelta > .25) {
                        m_timer.reset();
                        m_timer.start();
                    }
                }
                break;
        }
        if (currPose.getX() < centerLine && robot.getRed() && xspd < 0){
            xspd = 0;
        }
        if (currPose.getX() > centerLine && !robot.getRed() && xspd > 0){
            xspd = 0;
        }
        if (autonomousStep > 0) turnspd = PID.calculate(-robot.getYaw());
        robot.raiseTopRoller(collectorUp);
        robot.setDrive(xspd, yspd, turnspd, true, true);
        robot.collect(collectorRPM);
        robot.openGripper(openGripper);
        robot.setLightsOn(lightOn);
        robot.holdArmPosition(armPos);

    }

    @Override
    public double positionFunctionX(double s) {
        if (autonomousStep == 1){
            return AutoCodeLines.getPositionX(startPosition, firstScorePosition, s);
        }
        else{
            return AutoCodeLines.getPositionX(estimatedCubeSpot, spotBeforeEngage, s);
        }
    }

    @Override
    public double positionFunctionY(double s) {
        if (autonomousStep == 1){
            return AutoCodeLines.getPositionY(startPosition, firstScorePosition, s);
        }
        else{
            return AutoCodeLines.getPositionY(estimatedCubeSpot, spotBeforeEngage, s);
        }
    }


    @Override
    public double velocityFunctionX(double s, double time) {
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityX(startPosition, firstScorePosition, s) * getdS(time);
        }
        else{
            return AutoCodeLines.getVelocityX(estimatedCubeSpot, spotBeforeEngage, s) * getdS(time);
        }
    }

    @Override
    public double velocityFunctionY(double s, double time) {
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityX(startPosition, firstScorePosition, s) * getdS(time);
        }
        else{
            return AutoCodeLines.getVelocityX(estimatedCubeSpot, spotBeforeEngage, s) * getdS(time);
        }
    }


    @Override
    public double getS(double time) {
        if (autonomousStep == 1){
            return AutoCodeLines.getS(dist1, .1, 20, time);
        }
        else{
            return AutoCodeLines.getS(dist2, .1, 40, time);
        }
    }

    @Override
    public double getdS(double time) {
        if (autonomousStep == 1){
            return AutoCodeLines.getdS(dist1, .1, 20, time);
        }
        else{
            return AutoCodeLines.getdS(dist2, .1, 40, time);
        }
    }
}
