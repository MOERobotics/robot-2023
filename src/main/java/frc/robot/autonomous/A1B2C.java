package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import frc.robot.vision.Detection;
import frc.robot.vision.MoeNetVision;
import org.opencv.core.Point;

public class A1B2C extends genericAutonomous {
    double TARGET_DISTANCE = 0; //this is distance of camera
    double xspd, yspd, turnspd;
    double desiredInchesPerSecond = 70;
    double xPidK = 7;
    double yPidK = 7;
    double defaultSpeed = 20;
    boolean autoMode = false;
    double kP = 1.0e-1; //.5e-1
    double s = 0;
    PIDController PID = new PIDController(kP, 0, 0);
    private final Timer m_timer = new Timer();
    ////////////////////////////////////////////////////////////////////////////////////////////////Point stuff
    Point positionA = new Point(55.88+4, 195.47);
    Point position1Object = new Point(250.88, 190.235);
    Point position1 = new Point(275.88, 186.235);
    Point positionB = new Point(65.88, 186.06);
    Point positionInterMedFwd = new Point(190.76, 179.43);
    Point position2Object = new Point(260, 145);
    Point position2 = new Point(276.88, 137.24);
    Point positionInterMedBwd = new Point(190.76, 179.43);
    Point positionC = new Point(71.76, 167.36);

    Point positionABlue = new Point(55.88+4, 195.47);
    Point position1ObjectBlue = new Point(250.88, 190.235);
    Point position1Blue = new Point(275.88, 186.235);
    Point positionBBlue = new Point(65.88, 186.06);
    Point positionInterMedFwdBlue = new Point(190.76, 179.43);
    Point position2ObjectBlue = new Point(260,145);
    Point position2Blue = new Point(276.88, 137.24);
    Point positionInterMedBwdBlue = new Point(190.76, 179.43);
    Point positionCBlue = new Point(71.76, 167.36);

    double distAto1 = AutoCodeLines.getDistance(positionA,position1Object);
    double dist1toB = AutoCodeLines.getDistance(position1,positionB);
    double distBtoInterMed = AutoCodeLines.getDistance(positionB,positionInterMedFwd);
    double distInterMedto2 = AutoCodeLines.getDistance(positionInterMedFwd,position2Object);
    double dist2toInterMed = AutoCodeLines.getDistance(position2,positionInterMedBwd);
    double distInterMedtoC= AutoCodeLines.getDistance(positionInterMedBwd,positionC);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double lengthOfField = 650.7;
    boolean collectorUp = true;
    double collectorRPM = 0;
    boolean openGripper = true;
    double armPos = 0;
    double centerlineBlue = 300;
    double centerLine = centerlineBlue;
    Rotation2d startRot;
    MoeNetVision vision;
    Pose2d desiredPose1;
    Pose2d desiredPose2;


    public void autonomousInit(GenericRobot robot) {
        vision = new MoeNetVision(robot);
        startRot = new Rotation2d(0);
        PID.enableContinuousInput(-180,180);
        autonomousStep = 0;
        autoMode = false;
        robot.setPigeonYaw(0);
        if(robot.getRed()){
            positionA.x = lengthOfField - positionABlue.x;
            position1Object.x = lengthOfField - position1ObjectBlue.x;
            position1.x = lengthOfField - position1Blue.x;
            positionB.x = lengthOfField - positionBBlue.x;
            positionInterMedFwd.x = lengthOfField - positionInterMedFwdBlue.x;
            position2Object.x = lengthOfField - position2ObjectBlue.x;
            position2.x = lengthOfField - position2Blue.x;
            positionInterMedBwd.x = lengthOfField - positionInterMedBwdBlue.x;
            positionC.x = lengthOfField - positionCBlue.x;
            centerLine = lengthOfField - centerlineBlue;
            startRot = new Rotation2d(Math.PI);
            robot.setPigeonYaw(180);
        }else{
            positionA.x = positionABlue.x;
            position1Object.x = position1ObjectBlue.x;
            position1.x = position1Blue.x;
            positionB.x = positionBBlue.x;
            positionInterMedFwd.x = positionInterMedFwdBlue.x;
            position2Object.x = position2ObjectBlue.x;
            position2.x = position2Blue.x;
            positionInterMedBwd.x = positionInterMedBwdBlue.x;
            positionC.x = positionCBlue.x;
            centerLine = centerlineBlue;

        }
        desiredPose1 = new Pose2d(position1.x, position1.y, startRot);
        desiredPose2 = new Pose2d(position2.x, position2.y, startRot);
        robot.setPose(new Pose2d(positionA.x, positionA.y, startRot));
        robot.resetStartHeading();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetAttitude();
        autonomousStep = 0;
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
        SmartDashboard.putBoolean("do I think im red?", robot.getRed());
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
                if (m_timer.get() > 1){
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
                    this.desiredPose1 = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    SmartDashboard.putString("detautoTarget", String.format("%f, %f", this.desiredPose1.getX(), this.desiredPose1.getY()));
                }
/////////////////////////////////////////////////////////////////////////////////vision detection code
                if (s >= distAto1) {
                    xspd = 0;
                    yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
            case 2: ///object detection step
                firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                            .times(Units.metersToInches(1));
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);
                    this.desiredPose1 = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    SmartDashboard.putString("detautoTarget", String.format("%f, %f", this.desiredPose1.getX(), this.desiredPose1.getY()));
                }

                double xDiff = desiredPose1.getX()-currPose.getX() + 6;
                double yDiff = desiredPose1.getY()-currPose.getY();
                double totDiff = Math.hypot(xDiff, yDiff);
                xspd = yspd = 0;
                if (totDiff > 0) xspd = defaultSpeed * xDiff/totDiff;
                if (totDiff > 0) yspd = defaultSpeed * yDiff/totDiff;
                if (robot.cargoDetected() || m_timer.get() > 2){
                    position1 = new Point(currPose.getX(), currPose.getY());
                    dist1toB = AutoCodeLines.getDistance(position1, positionB);
                    collectorRPM = 4000;
                    xspd = yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
            case 3: //drive to position B
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoDetected()) collectorRPM = 4000;
                if (s >= dist1toB) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 4: //spit it out
                xspd = yspd = turnspd = 0;
                autoMode = true;
                collectorRPM = 9000;
                armPos = 20;
                if (m_timer.get() >= 1){
                    autonomousStep += 1;
                    m_timer.reset();
                    m_timer.start();
                }
                break;
            case 5: //go to intermediary point
                autoMode = false;
                collectorRPM = 0;
                armPos = -4;
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= distBtoInterMed) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 6: //go to spot before object detection
                t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s, t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoDetected()) collectorRPM = 4000;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                            .times(Units.metersToInches(1));
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);
                    this.desiredPose2 = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    SmartDashboard.putString("detautoTarget", String.format("%f, %f", this.desiredPose2.getX(), this.desiredPose2.getY()));
                }
/////////////////////////////////////////////////////////////////////////////////vision detection code
                if (s >= distInterMedto2) {
                    xspd = 0;
                    yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
            case 7: //object detection again
                firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
                if(firstDetection != null){
                    var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                            .times(Units.metersToInches(1));
                    double distance = objOffset.getNorm();
                    var targetPosition = objOffset.interpolate(new Translation2d(), 1-(distance-TARGET_DISTANCE)/distance);
                    this.desiredPose2 = currPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                    SmartDashboard.putString("detautoTarget", String.format("%f, %f", this.desiredPose2.getX(), this.desiredPose2.getY()));
                }

                xDiff = desiredPose2.getX()-currPose.getX() + 6;
                yDiff = desiredPose2.getY()-currPose.getY();
                totDiff = Math.hypot(xDiff, yDiff);
                xspd = yspd = 0;
                if (totDiff > 0) xspd = defaultSpeed * xDiff/totDiff;
                if (totDiff > 0) yspd = defaultSpeed * yDiff/totDiff;
                if (robot.cargoDetected() || m_timer.get() > 2){
                    position2 = new Point(currPose.getX(), currPose.getY());
                    dist2toInterMed = AutoCodeLines.getDistance(position2, positionInterMedBwd);
                    collectorRPM = 4000;
                    xspd = yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep ++;
                }
                break;
            case 8: //go to Intermediate from 2
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (robot.cargoDetected()) collectorRPM = 4000;
                if (s >= dist2toInterMed) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 9: //go to spot C from intermediary
                t = m_timer.get();
                s = getS(m_timer.get());
                xspd = velocityFunctionX(s, t)+ xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s, t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= distInterMedtoC) {
                    xspd = 0;
                    yspd = 0;
                    turnspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 10: //spit it out
                xspd = yspd = turnspd = 0;
                autoMode = true;
                collectorRPM = 9000;
                armPos = 20;
                break;

        }
        turnspd = PID.calculate(-robot.getYaw());
        robot.raiseTopRoller(collectorUp);
        robot.setDrive(xspd, yspd, turnspd, true);
        robot.collect(collectorRPM, autoMode);
        robot.openGripper(openGripper);
        robot.holdArmPosition(armPos);


    }

    public double positionFunctionX(double s) {
        if (autonomousStep == 0) {
            return positionA.x;
        }
        if (autonomousStep == 1) {
            return AutoCodeLines.getPositionX(positionA, position1Object, s);
        }
        if (autonomousStep == 3) {
            return AutoCodeLines.getPositionX(position1, positionB, s);
        }
        if (autonomousStep == 5) {
            return AutoCodeLines.getPositionX(positionB, positionInterMedFwd, s);
        }
        if (autonomousStep == 6) {
            return AutoCodeLines.getPositionX(positionInterMedFwd, position2Object, s);
        }
        if (autonomousStep == 8){
            return AutoCodeLines.getPositionX(position2, positionInterMedBwd, s);
        }
        if (autonomousStep == 9){
            return AutoCodeLines.getPositionX(positionInterMedBwd, positionC, s);
        }
        return positionC.x;
    }

    @Override
    public double positionFunctionY(double s) {
        if (autonomousStep == 0) {
            return positionA.y;
        }
        if (autonomousStep == 1) {
            return AutoCodeLines.getPositionY(positionA, position1Object, s);
        }
        if (autonomousStep == 3) {
            return AutoCodeLines.getPositionY(position1, positionB, s);
        }
        if (autonomousStep == 5) {
            return AutoCodeLines.getPositionY(positionB, positionInterMedFwd, s);
        }
        if (autonomousStep == 6) {
            return AutoCodeLines.getPositionY(positionInterMedFwd, position2Object, s);
        }
        if (autonomousStep == 8){
            return AutoCodeLines.getPositionY(position2, positionInterMedBwd, s);
        }
        if (autonomousStep == 9){
            return AutoCodeLines.getPositionY(positionInterMedBwd, positionC, s);
        }
        return positionC.y;
    }

    public double velocityFunctionX(double s, double time) {
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityX(positionA, position1Object, s) * getdS(time);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getVelocityX(position1, positionB, s)* getdS(time);
        }
        if (autonomousStep == 5){
            return AutoCodeLines.getVelocityX(positionB, positionInterMedFwd, s)* getdS(time);
        }
        if (autonomousStep == 6){
            return AutoCodeLines.getVelocityX(positionInterMedFwd, position2Object, s)* getdS(time);
        }
        if (autonomousStep == 8){
            return AutoCodeLines.getVelocityX(position2, positionInterMedBwd, s)* getdS(time);
        }
        if (autonomousStep == 9){
            return AutoCodeLines.getVelocityX(positionInterMedBwd, positionC, s)* getdS(time);
        }
        return 0;
    }

    @Override
    public double velocityFunctionY(double s, double time) {
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getVelocityY(positionA, position1Object, s) * getdS(time);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getVelocityY(position1, positionB, s)* getdS(time);
        }
        if (autonomousStep == 5){
            return AutoCodeLines.getVelocityY(positionB, positionInterMedFwd, s)* getdS(time);
        }
        if (autonomousStep == 6){
            return AutoCodeLines.getVelocityY(positionInterMedFwd, position2Object, s)* getdS(time);
        }
        if (autonomousStep == 8){
            return AutoCodeLines.getVelocityY(position2, positionInterMedBwd, s)* getdS(time);
        }
        if (autonomousStep == 9){
            return AutoCodeLines.getVelocityY(positionInterMedBwd, positionC, s)* getdS(time);
        }
        return 0;
    }

    @Override
    public double getS(double time){
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getS(distAto1, .5, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getS(dist1toB, .5, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 5){
            return AutoCodeLines.getS(distBtoInterMed, .2, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 6){
            return AutoCodeLines.getS(distInterMedto2, .2, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 8){
            return AutoCodeLines.getS(dist2toInterMed, .2, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 9){
            return AutoCodeLines.getS(distInterMedtoC, .2, desiredInchesPerSecond, time);
        }
        return 0;
    }
    @Override
    public double getdS(double time){
        if (autonomousStep == 0){
            return 0;
        }
        if (autonomousStep == 1){
            return AutoCodeLines.getdS(distAto1, .5, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 3){
            return AutoCodeLines.getdS(dist1toB, .5, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 5){
            return AutoCodeLines.getdS(distBtoInterMed, .2, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 6){
            return AutoCodeLines.getdS(distInterMedto2, .2, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 8){
            return AutoCodeLines.getdS(dist2toInterMed, .2, desiredInchesPerSecond, time);
        }
        if (autonomousStep == 9){
            return AutoCodeLines.getdS(distInterMedtoC, .2, desiredInchesPerSecond, time);
        }
        return 0;
    }

}
