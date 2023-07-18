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

public class A1B2CFast extends genericAutonomous {
    double TARGET_DISTANCE = 0; //this is distance of camera
    double xspd, yspd, turnspd;
    double desiredInchesPerSecond = 80;
    double xPidK = 7;
    double yPidK = 7;

    double defaultSpeed = 40;
    boolean autoMode = false;
    boolean lightOn = false;
    double armPower = 0;
    ////////////////////////////////////////////////////////////////////////////////////////////////Point stuff
    Point positionABlue = new Point(59.88 + 21, 195.47); //SCORE HERE
    Point position1Blue = new Point(176.88, 194.735+6);
    Point positionARevisitedBlue = new Point(64.88, 195.47); //SCORE HERE
    Point positionBBlue = new Point(64.88, 170.47+6);
    Point position2Blue = new Point(188.88, 200); //Changed value to give buffer when clipping CS
    Point positionFrogBlue = new Point(166.88,200); //this is an arbitrary point to avoid charging station
    Point positionBRevisitedBlue = new Point(59.88, 194);//SCORE AGAIN!!
    Point positionCBlue = new Point(59.88, 156.37);


    //TODO: Not sure if this needs to be fixed but, our positions B and C do not align with position A

    Point positionA = new Point(positionABlue.x, positionABlue.y);
    Point position1 = new Point(position1Blue.x, position1Blue.y);
    Point positionARevisited = new Point(positionARevisitedBlue.x, positionARevisitedBlue.y);
    Point positionB = new Point(positionBBlue.x, positionBBlue.y);
    Point position2 = new Point(position2Blue.x, position2Blue.y);
    Point positionFrog = new Point(positionFrogBlue.x, positionFrogBlue.y);
    Point positionBRevisited = new Point(positionBRevisitedBlue.x, positionBRevisitedBlue.y);
    Point positionC = new Point(positionCBlue.x, positionCBlue.y);

    double distAto1 = AutoCodeLines.getDistance(positionA, position1);
    double dist1toARevisted = AutoCodeLines.getDistance(position1, positionARevisited);
    double distARevisitedtoB = AutoCodeLines.getDistance(positionARevisited, positionB);
    double distBto2 = AutoCodeLines.getDistance(positionB, position2);
    double dist2toFrog = AutoCodeLines.getDistance(position2, positionFrog);
    double distFrogtoB = AutoCodeLines.getDistance(positionFrog, positionBRevisited);
    double distBtoC = AutoCodeLines.getDistance(positionBRevisited, positionC);

    double centerLineBlue = 295;
    double centerLine = centerLineBlue;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double kP = 1.0e-1; //.5e-1
    double s = 0;
    private final Timer m_timer = new Timer();
    double lengthOfField = 650.7;
    boolean collectorUp = true;
    double collectorRPM = 0;
    boolean openGripper = true;
    double armPos = 0;
    //3rd level: 101 and 85
    //2nd level: 87 and 70
    double scoringArmPos = 97;
    double scoringDownwardArmPos = 83;
    double rollBackDist = 19;
    Rotation2d startRot;
    MoeNetVision vision;
    PIDController PID = new PIDController(kP, 0, 0);
    Pose2d desiredPoseBlue = new Pose2d(275.88, 189.235, new Rotation2d(0)); //default pos
    Pose2d desiredPose = new Pose2d(desiredPoseBlue.getX(), desiredPoseBlue.getY(), desiredPoseBlue.getRotation());
    double cube2Pos = 189.235+48;
    double startX;
    boolean timeToWait = false;
    boolean tripHappened = false;
    double desiredYaw = 0;

    public void autonomousInit(GenericRobot robot) {
        armPower = 0;
        vision = new MoeNetVision(robot);
        startRot = new Rotation2d(0);
        autonomousStep = 0;
        lightOn = false;
        autoMode = false;
        robot.setPigeonYaw(0);
        if(robot.getRed()){
            startRot = new Rotation2d(Math.PI);
            positionA.x = lengthOfField - positionABlue.x;
            position1.x = lengthOfField - position1Blue.x;
            positionARevisited.x = lengthOfField - positionARevisitedBlue.x;
            positionB.x = lengthOfField - positionBBlue.x;
            position2.x = lengthOfField - position2Blue.x;
            positionFrog.x = lengthOfField - positionFrogBlue.x;
            positionBRevisited.x = lengthOfField - positionBRevisitedBlue.x;
            desiredPose = new Pose2d(lengthOfField - desiredPoseBlue.getX(),
                    desiredPoseBlue.getY(), startRot);
            centerLine = lengthOfField - centerLineBlue;
            positionC.x = lengthOfField - positionCBlue.x;
            robot.setPigeonYaw(180);
        }else{
            positionA.x = positionABlue.x;
            position1.x = position1Blue.x;
            positionARevisited.x = positionARevisitedBlue.x;
            positionB.x = positionBBlue.x;
            position2.x = position2Blue.x;
            positionFrog.x = positionFrogBlue.x;
            positionBRevisited.x = positionBRevisitedBlue.x;
            positionC.x = positionCBlue.x;
            desiredPose = new Pose2d(desiredPoseBlue.getX(),
                    desiredPoseBlue.getY(), startRot);
        }
        robot.resetStartHeading();
        m_timer.reset();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetAttitude();
        robot.setPose(new Pose2d(positionA.x, positionA.y, startRot));
        autonomousStep = 0;
        PID.enableContinuousInput(-180,180);
        xspd = yspd = 0;
        openGripper = true;
        m_timer.reset();
        m_timer.start();
        timeToWait = false;
        tripHappened = false;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        Pose2d currPose = robot.getPose();
        SmartDashboard.putNumber("s", s);
        SmartDashboard.putNumber("autoStep", autonomousStep);
        SmartDashboard.putNumber("xpsd", xspd);
        SmartDashboard.putNumber("yspd", yspd);
        SmartDashboard.putNumber("turn speed", turnspd);
        SmartDashboard.putNumber("first distance", distAto1);
        SmartDashboard.putBoolean("do I think im red?", robot.getRed());
        SmartDashboard.putNumber("startPosition", positionA.x);
        SmartDashboard.putNumber("startBlue", positionABlue.x);
        SmartDashboard.putNumber("x correction", xPidK * (positionFunctionX(s) - currPose.getX()));
        SmartDashboard.putNumber("y correction", yPidK * (positionFunctionY(s) - currPose.getY()));
        SmartDashboard.putBoolean("tripHappened", tripHappened);
        armPower = 0;
        switch (autonomousStep) {
            case 0: //resets everything
                collectorUp = false;
                desiredYaw = 0;
                openGripper = false;
                collectorRPM = 0;
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();

                robot.setPose(new Pose2d(positionA.x, positionA.y, startRot));
                xspd = yspd = turnspd = 0;
                autonomousStep ++;
                autoMode = false;
                m_timer.reset();
                m_timer.get();
                startX = currPose.getX();
                break;
            case 1: //lift your arm
                xspd = 0;
                if (robot.getRed()) xspd *= -1;
                yspd = 0;
                if(Math.abs(currPose.getX() - startX) <= 1){
                    xspd = 0;
                    armPos = scoringArmPos;
                    if (Math.abs(robot.getPotDegrees() - armPos) <= 10){
                        m_timer.restart();
                        autonomousStep ++;
                    }
                }
                break;
            case 2: // go back and drop that cone!
                xspd = -60;
                if (robot.getRed()) xspd *= -1;
                yspd = 0;
                if(Math.abs(currPose.getX() - startX) >= rollBackDist){
                    xspd = 0;
                    openGripper = true;
                    armPos = scoringDownwardArmPos;
                    m_timer.restart();
                    autonomousStep = 20;
                }
                break;
            case 20:
                if (m_timer.get() > .2){
                    positionA = new Point(currPose.getX(), currPose.getY());
                    distAto1 = AutoCodeLines.getDistance(positionA, position1);
                    m_timer.restart();
                    autonomousStep = 3;
                }
                break;
            case 3: //drive to spot A and look for cube

                collectorRPM = 9000;
                collectorUp = false;
                autoMode = false;
                double t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                Detection firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                            .times(Units.metersToInches(1));
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);
                    Pose2d possPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    if (possPose.getX() > centerLine && robot.getRed()){
                        lightOn = true;
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    }
                    if (possPose.getX() < centerLine && !robot.getRed()){
                        lightOn = true;
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    }
                    SmartDashboard.putString("detautoTarget", String.format("%f, %f", this.desiredPose.getX(), this.desiredPose.getY()));
                }
/////////////////////////////////////////////////////////////////////////////////vision detection code
                if (s >= 20){
                    armPos = -6;
                }
                if (s >= distAto1) {
                    xspd = 0;
                    yspd = 0;
                    m_timer.restart();
                    autonomousStep = 4;
                }
                break;
            case 4: ///object detection step + pickup
                armPower = -.3;
                firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                            .times(Units.metersToInches(1));
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);
                    Pose2d possPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    if (possPose.getX() > centerLine && robot.getRed()){
                        lightOn = true;
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    }
                    if (possPose.getX() < centerLine && !robot.getRed()){
                        lightOn = true;
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
                if (robot.cargoDetected() || m_timer.get() > 4 || robot.cargoInCollector()){
                    timeToWait = true;
                    if (robot.cargoDetected()) tripHappened = true;
                    if (robot.cargoInCollector()) openGripper = false;
                    position1 = new Point(currPose.getX(), currPose.getY());
                    dist1toARevisted = AutoCodeLines.getDistance(position1, positionARevisited);
                    //reads new currPose and creates new path based on it
                    xspd = yspd = 0;
                    m_timer.restart();
                    autonomousStep++;
                }
                break;
            case 5://drive from detected spot to spot A
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoDetected()) collectorRPM = 9000;
                if (robot.cargoInCollector()) openGripper = false;
                if (s >= dist1toARevisted) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 6: //slide from spot A to spot B
                desiredPose = new Pose2d(desiredPose.getX(), cube2Pos, startRot);
                armPos = -4;
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX()) ;
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoInCollector()) openGripper = false;
                if (s >= distARevisitedtoB) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                    autoMode = true;
                    collectorRPM = 9000;
                }
                break;
            case 7: //spit cube out :) --> B lower zone
                xspd = yspd = turnspd = 0;
                autoMode = true;
                armPos = 25;
                collectorRPM = 9000;
                if (m_timer.get() >= .5){
                    openGripper = true;
                    autonomousStep ++;
                    m_timer.reset();
                    m_timer.start();
                }
                break;
            case 8: //wait for arm to go back into position
                collectorRPM = 9000;
                armPos = -4;
                xspd = yspd = 0;
                lightOn = false;
                if (m_timer.get() >= .1){
                    m_timer.restart();
                    autonomousStep++;
                }
                break;
            case 9: //drive to intermediary point, avoid CS! look for cube as you go
                armPower = -.3;
                armPos = -4;
                autoMode = false;
                t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                            .times(Units.metersToInches(1));
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);
                    Pose2d possPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    if (possPose.getX() > centerLine && robot.getRed()){
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                        lightOn = true;
                    }
                    if (possPose.getX() < centerLine && !robot.getRed()){
                        lightOn = true;
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    }
                }
                if (s >= distBto2) {
                    xspd = 0;
                    yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    desiredYaw = 40;
                    if (robot.getRed()) desiredYaw*= -1;
                    autonomousStep ++;
                }
                break;
            case 10: //Look for cube
                armPower = -.3;
                firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                            .times(Units.metersToInches(1));
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);
                    Pose2d possPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    if (possPose.getX() > centerLine && robot.getRed()){
                        lightOn = true;
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    }
                    if (possPose.getX() < centerLine && !robot.getRed()){
                        lightOn = true;
                        this.desiredPose = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    }
                    SmartDashboard.putString("detautoTarget", String.format("%f, %f", this.desiredPose.getX(), this.desiredPose.getY()));
                }

                xDiff = desiredPose.getX()-currPose.getX() + 6;
                yDiff = desiredPose.getY()-currPose.getY();
                totDiff = Math.hypot(xDiff, yDiff);
                xspd = yspd = 0;
                if (totDiff > 0) xspd = defaultSpeed * xDiff/totDiff;
                if (totDiff > 0) yspd = defaultSpeed * yDiff/totDiff;
                if (robot.cargoDetected() || m_timer.get() > 6 || robot.cargoInCollector()){
                    position2 = new Point(currPose.getX(), currPose.getY());
                    dist2toFrog = AutoCodeLines.getDistance(position2, positionFrog);
                    xspd = yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
            case 11: //go from obj pose to frog. Avoid CS!
                armPower = -.3;
                desiredYaw = 0;
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoInCollector()) openGripper = false;
                if (s >= dist2toFrog) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 12: //drive back from frog to zone B
                armPower = -.3;
                if (robot.cargoInCollector()) openGripper = false;
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= distFrogtoB) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 13: //go to C
                armPower = -.3;
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= distBtoC) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 14: // move arm out
                xspd = yspd = turnspd = 0;
                autoMode = true;
                armPos = 25;
                collectorRPM = 9000;
                if (m_timer.get() >= .2){
                    autonomousStep ++;
                    m_timer.reset();
                    m_timer.start();
                }
                break;
            case 15: //Deposit cube in lower zone C
                openGripper = true;
                xspd = 0;
                if (robot.getRed()) xspd = -xspd;
                if (m_timer.get() >= .8){
                    xspd = yspd = 0;
                }
                break;
            case 16: //stop everything and put arm in
                collectorRPM = 0;
                xspd = yspd = 0;
                armPos = -4;
                break;

        }
        if (autonomousStep > 0) turnspd = PID.calculate(desiredYaw-robot.getYaw());
        robot.raiseTopRoller(collectorUp);
        robot.setDrive(xspd, yspd, turnspd, true, true);
        robot.collect(collectorRPM, autoMode);
        robot.openGripper(openGripper);
        robot.setLightsOn(lightOn);
        if (armPower != 0){
            robot.moveArm(armPower);
        }
        else{
            robot.holdArmPosition(armPos);
        }


    }

    public double positionFunctionX(double s) {
        if (autonomousStep == 0)   return positionA.x;
        if (autonomousStep == 3)   return AutoCodeLines.getPositionX(positionA, position1, s);
        if (autonomousStep == 5)   return AutoCodeLines.getPositionX(position1, positionARevisited, s);
        if (autonomousStep == 6)   return AutoCodeLines.getPositionX(positionARevisited, positionB, s);
        if (autonomousStep == 9)   return AutoCodeLines.getPositionX(positionB, position2, s);
        if (autonomousStep == 11) return AutoCodeLines.getPositionX(position2, positionFrog, s);
        if (autonomousStep == 12) return AutoCodeLines.getPositionX(positionFrog, positionBRevisited, s);
        if (autonomousStep == 13) return AutoCodeLines.getPositionX(positionBRevisited, positionC, s);

        return positionBRevisited.x;
    }

    @Override
    public double positionFunctionY(double s) {
        if (autonomousStep == 0) return positionA.y;
        if (autonomousStep == 3) return AutoCodeLines.getPositionY(positionA, position1, s);
        if (autonomousStep == 5) return AutoCodeLines.getPositionY(position1, positionARevisited, s);
        if (autonomousStep == 6) return AutoCodeLines.getPositionY(positionARevisited, positionB, s);
        if (autonomousStep == 9) return AutoCodeLines.getPositionY(positionB, position2, s);
        if (autonomousStep == 11) return AutoCodeLines.getPositionY(position2, positionFrog, s);
        if (autonomousStep == 12) return AutoCodeLines.getPositionY(positionFrog, positionBRevisited, s);
        if (autonomousStep == 13) return AutoCodeLines.getPositionY(positionBRevisited, positionC, s);

        return positionBRevisited.y;
    }

    public double velocityFunctionX(double s, double time) {
        if (autonomousStep == 0) return 0;
        if (autonomousStep == 3) return AutoCodeLines.getVelocityX(positionA, position1, s) * getdS(time);
        if (autonomousStep == 5) return AutoCodeLines.getVelocityX(position1, positionARevisited, s) * getdS(time);
        if (autonomousStep == 6) return AutoCodeLines.getVelocityX(positionARevisited, positionB, s) * getdS(time);
        if (autonomousStep == 9) return AutoCodeLines.getVelocityX(positionB, position2, s) * getdS(time);
        if (autonomousStep == 11) return AutoCodeLines.getVelocityX(position2, positionFrog, s) * getdS(time);
        if (autonomousStep == 12) return AutoCodeLines.getVelocityX(positionFrog, positionBRevisited, s) * getdS(time);
        if (autonomousStep == 13) return AutoCodeLines.getVelocityX(positionBRevisited, positionC, s) * getdS(time);

        return 0;
    }



    @Override
    public double velocityFunctionY(double s, double time) {
        if (autonomousStep == 0) return 0;
        if (autonomousStep == 3)return AutoCodeLines.getVelocityY(positionA, position1, s)* getdS(time);
        if (autonomousStep == 5)return AutoCodeLines.getVelocityY(position1, positionARevisited, s)* getdS(time);
        if (autonomousStep == 6) return AutoCodeLines.getVelocityY(positionARevisited, positionB, s)* getdS(time);
        if (autonomousStep == 9) return AutoCodeLines.getVelocityY(positionB, position2, s)* getdS(time);
        if (autonomousStep == 11) return AutoCodeLines.getVelocityY(position2, positionFrog, s)* getdS(time);
        if (autonomousStep == 12) return AutoCodeLines.getVelocityY(positionFrog, positionBRevisited, s)* getdS(time);
        if (autonomousStep == 13) return AutoCodeLines.getVelocityY(positionBRevisited, positionC, s) * getdS(time);
        return 0;
    }

    @Override
    public double getS(double time){
        if (autonomousStep == 0)return 0;
        if (autonomousStep == 3) return AutoCodeLines.getS(distAto1, .75, desiredInchesPerSecond, time);
        if (autonomousStep == 5) return AutoCodeLines.getS(dist1toARevisted, .75, desiredInchesPerSecond, time);
        if (autonomousStep == 6) return AutoCodeLines.getS(distARevisitedtoB, .15, desiredInchesPerSecond-20, time);
        if (autonomousStep == 9) return AutoCodeLines.getS(distBto2, .25, desiredInchesPerSecond, time);
        if (autonomousStep == 11) return AutoCodeLines.getS(dist2toFrog, .25, desiredInchesPerSecond, time);
        if (autonomousStep == 12) return AutoCodeLines.getS(distFrogtoB, .25, desiredInchesPerSecond, time);
        if (autonomousStep == 13) return AutoCodeLines.getS(distBtoC, .15, desiredInchesPerSecond-20, time);
        return 0;
    }
    @Override
    public double getdS(double time){
        if (autonomousStep == 0) return 0;
        if (autonomousStep == 3) return AutoCodeLines.getdS(distAto1, .75, desiredInchesPerSecond, time);
        if (autonomousStep == 5) return AutoCodeLines.getdS(dist1toARevisted, .75, desiredInchesPerSecond, time);
        if (autonomousStep == 6) return AutoCodeLines.getdS(distARevisitedtoB, .15, desiredInchesPerSecond-20, time);
        if (autonomousStep == 9) return AutoCodeLines.getdS(distBto2, .25, desiredInchesPerSecond, time);
        if (autonomousStep == 11) return AutoCodeLines.getdS(dist2toFrog, .25, desiredInchesPerSecond, time);
        if (autonomousStep == 12) return AutoCodeLines.getdS(distFrogtoB, .25, desiredInchesPerSecond, time);
        if (autonomousStep == 13) return AutoCodeLines.getdS(distBtoC, .15, desiredInchesPerSecond-20, time);
        return 0;

    }
}