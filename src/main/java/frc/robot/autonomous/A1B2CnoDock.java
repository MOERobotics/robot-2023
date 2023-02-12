package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import org.opencv.core.Point;

public class A1B2CnoDock extends genericAutonomous{
    private final Timer m_timer = new Timer();

    Point positionA = new Point(71.76, 214.73);
    Point position1 = new Point(276.88, 182.24);
    Point positionB = new Point(71.76, 189.06);
    Point position2 = new Point(276.88, 137.24);
    Point positionC = new Point(71.76, 167.36);
    Point positionFrog = new Point(190.76, 179.43);

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

    @Override
    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.resetStartHeading();
        if(robot.getRed()){
            robot.setPose(new Pose2d(lengthOfField - positionA.x, positionA.y, new Rotation2d(0)));
        } else {
            robot.setPose(new Pose2d(positionA.x, positionA.y, new Rotation2d(0)));
        }
        autonomousStep = 0;
    }
    @Override
    public void autonomousPeriodic(GenericRobot robot){
        SmartDashboard.putNumber("autostep", autonomousStep);
        SmartDashboard.putNumber("Distance a to 1", distAto1);
        switch(autonomousStep){
            case 0:
                m_timer.reset();
                m_timer.start();
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                autonomousStep++;
                break;
            case 1:
                double s_0 = getS(m_timer.get());
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                turnspd = velocityFunctionTheta(s_0);
                SmartDashboard.putNumber("s in case 1", s_0);
                if (s_0 >= distAto1) {
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 2:
                double t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                turnspd = velocityFunctionTheta(s_0);
                if (s_0 >= dist1toB) {
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 3:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                turnspd = velocityFunctionTheta(s_0);
                if (s_0 >= distBtoFrog) {
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 4:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                turnspd = velocityFunctionTheta(s_0);
                if (s_0 >= distFrogto2) {
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 5:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                turnspd = velocityFunctionTheta(s_0);
                if (s_0 >= dist2toFrog) {
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 6:
                t = m_timer.get();
                s_0 = getS(t);
                xspd = velocityFunctionX(s_0);
                yspd = velocityFunctionY(s_0);
                turnspd = velocityFunctionTheta(s_0);
                if (s_0 >= distFrogtoC) {
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 7:
                m_timer.reset();
                xspd = yspd = turnspd = 0;
                break;
        }
        if(robot.getRed()){
            yspd = -yspd;
        }
        robot.setDrive(xspd, yspd, turnspd);
    }

    @Override
    public double positionFunctionX(double s){
        if (autonomousStep == 0){
            return positionA.x*ds;
        }
        else if (autonomousStep == 1){
            return AutoCodeLines.getPositionX(positionA, position1, s)*ds;
        }
        else if (autonomousStep == 2){
            return AutoCodeLines.getPositionX(position1, positionB, s)*ds;
        }
        else if (autonomousStep == 3){
            return AutoCodeLines.getPositionX(positionB, positionFrog, s)*ds;
        }
        else if (autonomousStep == 4){
            return AutoCodeLines.getPositionX(positionFrog, position2, s)*ds;
        }
        else if (autonomousStep == 5){
            return AutoCodeLines.getPositionX(position2, positionFrog, s)*ds;
        }
        else if (autonomousStep == 6){
            return AutoCodeLines.getPositionX(positionFrog, positionC, s)*ds;
        }
        else{
            return positionC.x*ds;
        }
    }

    @Override
    public double positionFunctionY(double s){
        if (autonomousStep == 0){
            return positionA.y*ds;
        }
        else if (autonomousStep == 1){
            return AutoCodeLines.getPositionY(positionA, position1, s)*ds;
        }
        else if (autonomousStep == 2){
            return AutoCodeLines.getPositionY(position1, positionB, s)*ds;
        }
        else if (autonomousStep == 3){
            return AutoCodeLines.getPositionY(positionB, positionFrog, s)*ds;
        }
        else if (autonomousStep == 4){
            return AutoCodeLines.getPositionY(positionFrog, position2, s)*ds;
        }
        else if (autonomousStep == 5){
            return AutoCodeLines.getPositionY(position2, positionFrog, s)*ds;
        }
        else if (autonomousStep == 6){
            return AutoCodeLines.getPositionY(positionFrog, positionC, s)*ds;
        }
        else{
            return positionC.y*ds;
        }
    }
    @Override
    public double positionFunctionTheta(double s){
        return 0*ds;
    }
    @Override
    public double velocityFunctionX(double s){
        if (autonomousStep == 0){
            return 0*ds;
        }
        else if (autonomousStep == 1){
            return AutoCodeLines.getVelocityX(positionA, position1, s)*ds;
        }
        else if (autonomousStep == 2){
            return AutoCodeLines.getVelocityX(position1, positionB, s)*ds;
        }
        else if (autonomousStep == 3){
            return AutoCodeLines.getVelocityX(positionB, positionFrog, s)*ds;
        }
        else if (autonomousStep == 4){
            return AutoCodeLines.getVelocityX(positionFrog, position2, s)*ds;
        }
        else if (autonomousStep == 5){
            return AutoCodeLines.getVelocityX(position2, positionFrog, s)*ds;
        }
        else if (autonomousStep == 6){
            return AutoCodeLines.getVelocityX(positionFrog, positionC, s)*ds;
        }
        else{
            return 0*ds;
        }
    }
    @Override
    public double velocityFunctionY(double s){
        if (autonomousStep == 0){
            return 0*ds;
        }
        else if (autonomousStep == 1){
            return AutoCodeLines.getVelocityY(positionA, position1, s)*ds;
        }
        else if (autonomousStep == 2){
            return AutoCodeLines.getVelocityY(position1, positionB, s)*ds;
        }
        else if (autonomousStep == 3){
            return AutoCodeLines.getVelocityY(positionB, positionFrog, s)*ds;
        }
        else if (autonomousStep == 4){
            return AutoCodeLines.getVelocityY(positionFrog, position2, s)*ds;
        }
        else if (autonomousStep == 5){
            return AutoCodeLines.getVelocityY(position2, positionFrog, s)*ds;
        }
        else if (autonomousStep == 6){
            return AutoCodeLines.getVelocityY(positionFrog, positionC, s)*ds;
        }
        else{
            return 0*ds;
        }
    }
    @Override
    public double velocityFunctionTheta(double s){
        return 0*ds;
    }
    @Override
    public double getS(double time){
        return time*ds;
    }
}
