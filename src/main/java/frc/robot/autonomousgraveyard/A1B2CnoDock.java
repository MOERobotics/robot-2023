package frc.robot.autonomousgraveyard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.genericAutonomous;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import org.opencv.core.Point;

public class A1B2CnoDock extends genericAutonomous {
    private final Timer m_timer = new Timer();

    //TODO: Verify
    Point positionA = new Point(71.76, 214.73);
    Point position1 = new Point(255.88, 182.24);
    Point positionB = new Point(71.76, 189.06);
    Point position2 = new Point(276.88, 137.24);
    Point positionC = new Point(71.76, 167.36);
    Point positionFrog = new Point(190.76, 179.43);

    Point positionABlue = new Point(71.76, 214.73);
    Point position1Blue = new Point(255.88, 182.24);
    Point positionBBlue = new Point(71.76, 189.06);
    Point position2Blue = new Point(276.88, 137.24);
    Point positionCBlue = new Point(71.76, 167.36);
    Point positionFrogBlue = new Point(190.76, 179.43);

    double distAto1 = AutoCodeLines.getDistance(positionA,position1);
    double dist1toB = AutoCodeLines.getDistance(position1,positionB);
    double distBtoFrog = AutoCodeLines.getDistance(positionB,positionFrog);
    double distFrogto2 = AutoCodeLines.getDistance(positionFrog,position2);
    double dist2toFrog = AutoCodeLines.getDistance(position2,positionFrog);
    double distFrogtoC= AutoCodeLines.getDistance(positionFrog,positionC);

    double inchesPerSecond = 24;
    double ds = inchesPerSecond;
    double xspd, yspd, turnspd;
    double lengthOfField = 650.7;
    double xPidK = 7;
    double yPidK = 7;
    double kP = 0.5e-1;
    double s = 0;
    PIDController PID = new PIDController(kP,0,0);

    double collectorRPM;
    double baseCollectionRPM = 7500;
    boolean dropTopRoller;

    @Override
    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetAttitude();
        robot.resetStartHeading();
        collectorRPM = 0;
        dropTopRoller = false;
        Rotation2d startRot = new Rotation2d(0);
        robot.setPigeonYaw(0);
        if(robot.getRed()){
            positionA.x = lengthOfField - positionABlue.x;
            position1.x = lengthOfField - position1Blue.x;
            positionB.x = lengthOfField - positionBBlue.x;
            position2.x = lengthOfField - position2Blue.x;
            positionC.x = lengthOfField - positionCBlue.x;
            positionFrog.x = lengthOfField - positionFrogBlue.x;

            startRot = new Rotation2d(Math.PI);
            robot.setPigeonYaw(180);
        } else {
            positionA.x = positionABlue.x;
            position1.x = position1Blue.x;
            positionB.x = positionBBlue.x;
            position2.x = position2Blue.x;
            positionC.x = positionCBlue.x;
            positionFrog.x = positionFrogBlue.x;
        }
        robot.setPose(new Pose2d(positionA.x, positionA.y, startRot));
        PID.enableContinuousInput(-180,180);
        xspd = yspd = 0;
        autonomousStep = 0;
    }
    @Override
    public void autonomousPeriodic(GenericRobot robot){
        SmartDashboard.putNumber("autostep", autonomousStep);
        SmartDashboard.putNumber("Distance a to 1", distAto1);

        Pose2d currPose = robot.getPose();

        switch(autonomousStep){
            case 0: //resets everything
                m_timer.reset();
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                autonomousStep++;
                break;
            case 1: //Drives from position A to position 1
                double t = m_timer.get();
                s = getS(m_timer.get());
                dropTopRoller = true;
                collectorRPM = baseCollectionRPM;
                xspd = velocityFunctionX(s,t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s,t) + yPidK * (positionFunctionY(s) - currPose.getY());
                SmartDashboard.putNumber("s in case 1", s);
                if (s >= distAto1) {
                    xspd = yspd = 0;
                    dropTopRoller = false;
                    collectorRPM = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 2: //Drives from position 1 to position B
                t = m_timer.get();
                s = getS(t);
                //TODO: add raising arm code
                xspd = velocityFunctionX(s,t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s,t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= dist1toB) {
                    xspd = yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 3: //Drives from poistion B to position Frog (to avoid charging station)
                t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s,t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s,t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= distBtoFrog) {
                    xspd = yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 4: //Drives from position Frog to position 2
                t = m_timer.get();
                s = getS(t);
                dropTopRoller = true;
                collectorRPM = baseCollectionRPM;
                xspd = velocityFunctionX(s,t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s,t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= distFrogto2) {
                    xspd = yspd = 0;
                    dropTopRoller = false;
                    collectorRPM = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 5: //Drives from position 2 to position Frog
                t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s,t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s,t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= dist2toFrog) {
                    xspd = yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 6: //Drives from position Frog to position C
                t = m_timer.get();
                s = getS(t);
                xspd = velocityFunctionX(s,t) + xPidK * (positionFunctionX(s) - currPose.getX());
                yspd = velocityFunctionY(s,t) + yPidK * (positionFunctionY(s) - currPose.getY());
                if (s >= distFrogtoC) {
                    xspd = yspd = 0;
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 7: //End of autonomous
                m_timer.reset();
                xspd = yspd = 0;
                break;
        }

        if(autonomousStep > 0) turnspd = PID.calculate(-robot.getYaw());
        robot.raiseTopRoller(dropTopRoller);
        robot.collect(collectorRPM);
        robot.setDrive(xspd, yspd, turnspd, true, true);
    }

    @Override
    public double positionFunctionX(double s){
        if (autonomousStep == 0){
            return positionA.x*ds;
        }
        else if (autonomousStep == 1){
            return AutoCodeLines.getPositionX(positionA, position1, s);
        }
        else if (autonomousStep == 2){
            return AutoCodeLines.getPositionX(position1, positionB, s);
        }
        else if (autonomousStep == 3){
            return AutoCodeLines.getPositionX(positionB, positionFrog, s);
        }
        else if (autonomousStep == 4){
            return AutoCodeLines.getPositionX(positionFrog, position2, s);
        }
        else if (autonomousStep == 5){
            return AutoCodeLines.getPositionX(position2, positionFrog, s);
        }
        else if (autonomousStep == 6){
            return AutoCodeLines.getPositionX(positionFrog, positionC, s);
        }
        else{
            return positionC.x;
        }
    }

    @Override
    public double positionFunctionY(double s){
        if (autonomousStep == 0){
            return positionA.y;
        }
        else if (autonomousStep == 1){
            return AutoCodeLines.getPositionY(positionA, position1, s);
        }
        else if (autonomousStep == 2){
            return AutoCodeLines.getPositionY(position1, positionB, s);
        }
        else if (autonomousStep == 3){
            return AutoCodeLines.getPositionY(positionB, positionFrog, s);
        }
        else if (autonomousStep == 4){
            return AutoCodeLines.getPositionY(positionFrog, position2, s);
        }
        else if (autonomousStep == 5){
            return AutoCodeLines.getPositionY(position2, positionFrog, s);
        }
        else if (autonomousStep == 6){
            return AutoCodeLines.getPositionY(positionFrog, positionC, s);
        }
        else{
            return positionC.y;
        }
    }

    @Override
    public double velocityFunctionX(double s, double time){
        if (autonomousStep == 0){
            return 0*ds;
        }
        else if (autonomousStep == 1){
            return AutoCodeLines.getVelocityX(positionA, position1, s)*getdS(time);
        }
        else if (autonomousStep == 2){
            return AutoCodeLines.getVelocityX(position1, positionB, s)*getdS(time);
        }
        else if (autonomousStep == 3){
            return AutoCodeLines.getVelocityX(positionB, positionFrog, s)*getdS(time);
        }
        else if (autonomousStep == 4){
            return AutoCodeLines.getVelocityX(positionFrog, position2, s)*getdS(time);
        }
        else if (autonomousStep == 5){
            return AutoCodeLines.getVelocityX(position2, positionFrog, s)*getdS(time);
        }
        else if (autonomousStep == 6){
            return AutoCodeLines.getVelocityX(positionFrog, positionC, s)*getdS(time);
        }
        else{
            return 0;
        }
    }
    @Override
    public double velocityFunctionY(double s, double time){
        if (autonomousStep == 0){
            return 0;
        }
        else if (autonomousStep == 1){
            return AutoCodeLines.getVelocityY(positionA, position1, s)*getdS(time);
        }
        else if (autonomousStep == 2){
            return AutoCodeLines.getVelocityY(position1, positionB, s)*getdS(time);
        }
        else if (autonomousStep == 3){
            return AutoCodeLines.getVelocityY(positionB, positionFrog, s)*getdS(time);
        }
        else if (autonomousStep == 4){
            return AutoCodeLines.getVelocityY(positionFrog, position2, s)*getdS(time);
        }
        else if (autonomousStep == 5){
            return AutoCodeLines.getVelocityY(position2, positionFrog, s)*getdS(time);
        }
        else if (autonomousStep == 6){
            return AutoCodeLines.getVelocityY(positionFrog, positionC, s)*getdS(time);
        }
        else{
            return 0;
        }
    }

    @Override
    public double getS(double time){
        if (autonomousStep == 0) {
            return 0;
        }
        if (autonomousStep == 1) {
            return AutoCodeLines.getS(distAto1,0.5,inchesPerSecond,time);
        }
        if (autonomousStep == 2) {
            return AutoCodeLines.getS(dist1toB,0.5,inchesPerSecond,time);
        }
        if (autonomousStep == 3) {
            return AutoCodeLines.getS(distBtoFrog,0.5,inchesPerSecond,time);
        }
        if (autonomousStep == 4) {
            return AutoCodeLines.getS(distFrogto2,0.5,inchesPerSecond,time);
        }
        if (autonomousStep == 5) {
            return AutoCodeLines.getS(dist2toFrog,0.5,inchesPerSecond,time);
        }
        if (autonomousStep == 6) {
            return AutoCodeLines.getS(distFrogtoC,0.5,inchesPerSecond,time);
        }
        return 0;
    }

    @Override
    public double getdS(double time) {
        if (autonomousStep == 0) {
            return 0;
        }
        if (autonomousStep == 1) {
            return AutoCodeLines.getdS(distAto1,0.5,inchesPerSecond,time);
        }
        if (autonomousStep == 2) {
            return AutoCodeLines.getdS(dist1toB,0.5,inchesPerSecond,time);
        }
        if (autonomousStep == 3) {
            return AutoCodeLines.getdS(distBtoFrog,0.5,inchesPerSecond,time);
        }
        if (autonomousStep == 4) {
            return AutoCodeLines.getdS(distFrogto2,0.5,inchesPerSecond,time);
        }
        if (autonomousStep == 5) {
            return AutoCodeLines.getdS(dist2toFrog,0.5,inchesPerSecond,time);
        }
        if (autonomousStep == 6) {
            return AutoCodeLines.getdS(distFrogtoC,0.5,inchesPerSecond,time);
        }
        return 0;
    }
}
