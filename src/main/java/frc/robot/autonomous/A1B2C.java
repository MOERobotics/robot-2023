package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.vision.Detection;
import frc.robot.vision.MoeNetVision;
import org.opencv.core.Point;
import frc.robot.helpers.AutoCodeLines;
import edu.wpi.first.math.geometry.Rotation2d;

public class A1B2C extends genericAutonomous {
    double TARGET_DISTANCE = 0; //this is distance of camera
    double xspd, yspd, turnspd;
    double desiredInchesPerSecond = 60;
    double xPidK = 7;
    double yPidK = 7;

    double defaultSpeed = 40;
    boolean autoMode = false;
    boolean lightOn = false;
    ////////////////////////////////////////////////////////////////////////////////////////////////Point stuff
    Point positionABlue = new Point(59.88, 195.47);
    Point position1Blue = new Point(188.88, 192.735);
    Point positionARevisitedBlue = new Point(64.88, 195.47);
    Point positionBBlue = new Point(64.88, 170.47);
    Point position2Blue = new Point(210.88, 170.47);
    Point positionFrogBlue = new Point(130.88,170.47); //this is an arbitrary point to avoid charging station
    Point positionCBlue = new Point(64.88, 148.37);

    //TODO: Not sure if this needs to be fixed but, our positions B and C do not align with position A

    Point positionA = new Point(positionABlue.x, positionABlue.y);
    Point position1 = new Point(position1Blue.x, position1Blue.y);
    Point positionARevisited = new Point(positionARevisitedBlue.x, positionARevisitedBlue.y);
    Point positionB = new Point(positionBBlue.x, positionBBlue.y);
    Point position2 = new Point(position2Blue.x, position2Blue.y);
    Point positionFrog = new Point(positionFrogBlue.x, positionFrogBlue.y);
    Point positionC = new Point(positionCBlue.x, positionCBlue.y);

    double distAto1 = AutoCodeLines.getDistance(positionA, position1);
    double dist1toARevisted = AutoCodeLines.getDistance(position1, positionARevisited);
    double distARevisitedtoB = AutoCodeLines.getDistance(positionARevisited, positionB);
    double distBto2 = AutoCodeLines.getDistance(positionB, position2);
    double dist2toFrog = AutoCodeLines.getDistance(position2, positionFrog);
    double distFrogtoC = AutoCodeLines.getDistance(positionFrog, positionC);

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
    Rotation2d startRot;
    MoeNetVision vision;
    PIDController PID = new PIDController(kP, 0, 0);
    Pose2d desiredPoseBlue = new Pose2d(275.88, 189.235, new Rotation2d(0)); //default pos
    Pose2d desiredPose = new Pose2d(desiredPoseBlue.getX(), desiredPoseBlue.getY(), desiredPoseBlue.getRotation());


    public void autonomousInit(GenericRobot robot) {
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
            positionC.x = lengthOfField - positionCBlue.x;
            desiredPose = new Pose2d(lengthOfField - desiredPoseBlue.getX(),
                    desiredPoseBlue.getY(), startRot);
            centerLine = lengthOfField - centerLineBlue;
            robot.setPigeonYaw(180);
        }else{
            positionA.x = positionABlue.x;
            position1.x = position1Blue.x;
            positionARevisited.x = positionARevisitedBlue.x;
            positionB.x = positionBBlue.x;
            position2.x = position2Blue.x;
            positionFrog.x = positionFrogBlue.x;
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
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        Pose2d currPose = robot.getPose();
        SmartDashboard.putNumber("s", s);
        SmartDashboard.putNumber("autostep", autonomousStep);
        SmartDashboard.putNumber("xpsd", xspd);
        SmartDashboard.putNumber("yspd", yspd);
        SmartDashboard.putNumber("turn speed", turnspd);
        SmartDashboard.putNumber("first distance", distAto1);
        SmartDashboard.putBoolean("do I think im red?", robot.getRed());
        SmartDashboard.putNumber("startPosition", positionA.x);
        SmartDashboard.putNumber("startBlue", positionABlue.x);
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
                robot.setPose(new Pose2d(positionA.x, positionA.y, startRot));
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
                if (s >= distAto1) {
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
                if (robot.cargoDetected() || m_timer.get() > 6){
                    //reads new currPose and creates new path based on it
                    position1 = new Point(currPose.getX(), currPose.getY());
                    dist1toARevisted = AutoCodeLines.getDistance(position1, positionARevisited);
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
                if (s >= dist1toARevisted) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 4:
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX()) ;
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= distARevisitedtoB) {
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
            case 6: //wait for arm to go back into position
                collectorRPM = 0;
                armPos = -4;
                xspd = yspd = 0;
                if (robot.getArmPosition() < 0){
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
            case 7:
                armPos = -4;
                autoMode = false;
                t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoDetected()) collectorRPM = 4000;
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
                }
                if (s >= distBto2) {
                    xspd = 0;
                    yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep += 1;
                }
                break;
            case 8:
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
                if (robot.cargoDetected() || m_timer.get() > 6){
                    position2 = new Point(currPose.getX(), currPose.getY());
                    dist2toFrog = AutoCodeLines.getDistance(position2, positionFrog);
                    collectorRPM = 4000;
                    xspd = yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep += 1;
                }
                break;
            case 9:
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoDetected()) collectorRPM = 4000;
                if (s >= dist2toFrog) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 10:
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoDetected()) collectorRPM = 4000;
                if (s >= distFrogtoC) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 11:
                xspd = yspd = turnspd = 0;
                autoMode = true;
                armPos = 45;
                collectorRPM = 9000;
                if (m_timer.get() >= 1.0){
                    autonomousStep ++;
                    m_timer.reset();
                    m_timer.start();
                }
                break;
            case 12: //I genuniely have no clue what this does, yes I know I spelt genueniely wrong but thats what I code and not write papers
                xspd = -12;
                if (robot.getRed()) xspd = -xspd;
                if (m_timer.get() >= .8){
                    xspd = yspd = 0;
                }
                break;

        }
        if (autonomousStep > 0) turnspd = PID.calculate(-robot.getYaw());
        robot.raiseTopRoller(collectorUp);
        robot.setDrive(xspd, yspd, turnspd, true, true);
        robot.collect(collectorRPM, autoMode);
        robot.openGripper(openGripper);
        robot.setLightsOn(lightOn);
        robot.holdArmPosition(armPos);


    }

    public double positionFunctionX(double s) {
        if (autonomousStep == 0)   return positionA.x;
        if (autonomousStep == 1)   return AutoCodeLines.getPositionX(positionA, position1, s);
        if (autonomousStep == 3)   return AutoCodeLines.getPositionX(position1, positionARevisited, s);
        if (autonomousStep == 4)   return AutoCodeLines.getPositionX(positionARevisited, positionB, s);
        if (autonomousStep == 7)   return AutoCodeLines.getPositionX(positionB, position2, s);
        if (autonomousStep == 9) return AutoCodeLines.getPositionX(position2, positionFrog, s);
        if (autonomousStep == 10) return AutoCodeLines.getPositionX(positionFrog, positionC, s);
        return positionC.x;
    }

    @Override
    public double positionFunctionY(double s) {
        if (autonomousStep == 0) return positionA.y;
        if (autonomousStep == 1) return AutoCodeLines.getPositionY(positionA, position1, s);
        if (autonomousStep == 3) return AutoCodeLines.getPositionY(position1, positionARevisited, s);
        if (autonomousStep == 4) return AutoCodeLines.getPositionY(positionARevisited, positionB, s);
        if (autonomousStep == 7) return AutoCodeLines.getPositionY(positionB, position2, s);
        if (autonomousStep == 9) return AutoCodeLines.getPositionY(position2, positionFrog, s);
        if (autonomousStep == 10) return AutoCodeLines.getPositionY(positionFrog, positionC, s);
        return positionC.y;
    }

    public double velocityFunctionX(double s, double time) {
        if (autonomousStep == 0) return 0;
        if (autonomousStep == 1) return AutoCodeLines.getVelocityX(positionA, position1, s) * getdS(time);
        if (autonomousStep == 3) return AutoCodeLines.getVelocityX(position1, positionARevisited, s) * getdS(time);
        if (autonomousStep == 4) return AutoCodeLines.getVelocityX(positionARevisited, positionB, s) * getdS(time);
        if (autonomousStep == 7) return AutoCodeLines.getVelocityX(positionB, position2, s) * getdS(time);
        if (autonomousStep == 9) return AutoCodeLines.getVelocityX(position2, positionFrog, s) * getdS(time);
        if (autonomousStep == 10) return AutoCodeLines.getVelocityX(positionFrog, positionC, s) * getdS(time);
        return 0;
    }



    @Override
    public double velocityFunctionY(double s, double time) {
        if (autonomousStep == 0) return 0;
        if (autonomousStep == 1)return AutoCodeLines.getVelocityY(positionA, position1, s)* getdS(time);
        if (autonomousStep == 3)return AutoCodeLines.getVelocityY(position1, positionARevisited, s)* getdS(time);
        if (autonomousStep == 4) return AutoCodeLines.getVelocityY(positionARevisited, positionB, s)* getdS(time);
        if (autonomousStep == 7) return AutoCodeLines.getVelocityY(positionB, position2, s)* getdS(time);
        if (autonomousStep == 9) return AutoCodeLines.getVelocityY(position2, positionFrog, s)* getdS(time);
        if (autonomousStep == 10) return AutoCodeLines.getVelocityY(positionFrog, positionC, s)* getdS(time);
        return 0;
    }

    @Override
    public double getS(double time){
        if (autonomousStep == 0)return 0;
        if (autonomousStep == 1) return AutoCodeLines.getS(distAto1, .55, desiredInchesPerSecond, time);
        if (autonomousStep == 3) return AutoCodeLines.getS(dist1toARevisted, .55, desiredInchesPerSecond, time);
        if (autonomousStep == 4) return AutoCodeLines.getS(distARevisitedtoB, .25, desiredInchesPerSecond-20, time);
        if (autonomousStep == 7) return AutoCodeLines.getS(distBto2, .25, desiredInchesPerSecond-20, time);
        if (autonomousStep == 9) return AutoCodeLines.getS(dist2toFrog, .25, desiredInchesPerSecond-20, time);
        if (autonomousStep == 10) return AutoCodeLines.getS(distFrogtoC, .25, desiredInchesPerSecond-20, time);
        return 0;
    }
    @Override
    public double getdS(double time){
        if (autonomousStep == 0) return 0;
        if (autonomousStep == 1) return AutoCodeLines.getdS(distAto1, .55, desiredInchesPerSecond, time);
        if (autonomousStep == 3) return AutoCodeLines.getdS(dist1toARevisted, .55, desiredInchesPerSecond, time);
        if (autonomousStep == 4) return AutoCodeLines.getdS(distARevisitedtoB, .25, desiredInchesPerSecond-20, time);
        if (autonomousStep == 7) return AutoCodeLines.getdS(distBto2, .25, desiredInchesPerSecond-20, time);
        if (autonomousStep == 9) return AutoCodeLines.getdS(dist2toFrog, .25, desiredInchesPerSecond-20, time);
        if (autonomousStep == 10) return AutoCodeLines.getdS(distFrogtoC, .25, desiredInchesPerSecond-20, time);
        return 0;
    }
}