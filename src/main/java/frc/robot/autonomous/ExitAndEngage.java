//THE FALLBACK AUTO - AUTO1

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class ExitAndEngage extends genericAutonomous{

    private final Timer m_timer = new Timer();
    boolean autoMode = false;
    double desiredInchesPerSecond = 12;
    double ds = desiredInchesPerSecond;
    double xspd, yspd, turnspd;
    double s_0;
    double currPitch;
    double currRoll;
    double desiredPitch = 9.0;
    double xPose;
    double baseSpd = 40.0;
    double correctionPower = -14.0;
    double climbPower = -30.0;
    double basePower = -45.0;
    double armPos = 20;
    double kP = 0.1;

    boolean collectorUp = true;
    double collectorRPM = 0;
    boolean openGripper = true;
    PIDController PID = new PIDController(kP,0,0);
    @Override
    public void autonomousInit(GenericRobot robot){
        m_timer.reset();
        m_timer.start();
        autonomousStep = 0;
        robot.resetAttitude();
        robot.resetPIDPivot();
        robot.resetStartHeading();
        robot.resetStartDists();
        robot.resetStartPivots();
        robot.setPose();
        PID.enableContinuousInput(-180,180);
    }
    @Override
    public void autonomousPeriodic(GenericRobot robot){

        currPitch = robot.getPitch(); //test switching roll and pitch
        currRoll = robot.getRoll();
        //I4H
        SmartDashboard.putNumber("autostep", autonomousStep);
        SmartDashboard.putNumber("s",s_0);
        Pose2d currPose = robot.getPose();
        switch(autonomousStep){
            case 0:
                armPos = 20;
                collectorRPM = 9000;
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                if(m_timer.get()>2) {
                    m_timer.reset();
                    m_timer.start();
                    autonomousStep++;
                }
                break;
            case 1:
                armPos = 0;
                xspd = baseSpd;
                collectorRPM = 0;
                yspd = 0;
                if (robot.getPitch() > 11) {//robot is on charge station
                    autonomousStep++;
                }
                break;
            case 2:
                xspd = baseSpd;
                if (robot.getPitch() < -11){ //robot is getting off of charge station
                    autonomousStep++;
                }
                break;
            case 3:
                xspd = baseSpd;
                if (currPitch < 9 && currPitch >=0) { //robot has exited charge station
                    xPose = currPose.getX();
                    autonomousStep++;
                }
                break;
            case 4:
                xspd = baseSpd;
                if (currPose.getX() - xPose >= 10) {// make sure you fully exit community
                    xspd = 0;
                    autonomousStep++;
                }
                break;
            case 5:
                xspd = basePower;
                if (Math.abs(currPitch) > 11) { // driving back up charge station
                    autonomousStep += 1;
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
                    xspd = correctionPower; //backward
                } else if (currPitch > desiredPitch) {
                    xspd = -correctionPower; //forward
                } else {
                    xspd = 0;
                }
                break;
        }
        turnspd = PID.calculate(-robot.getYaw());
        robot.setDrive(xspd, yspd, turnspd);
        robot.collect(collectorRPM, true);
        robot.holdArmPosition(armPos);
        robot.openGripper(false);
        robot.raiseTopRoller(collectorUp);
    }

    @Override
    public double positionFunctionX(double s) {
        return super.positionFunctionX(s);
    }

    @Override
    public double positionFunctionY(double s) {
        return super.positionFunctionY(s);
    }

    @Override
    public double positionFunctionTheta(double s) {
        return super.positionFunctionTheta(s);
    }

    @Override
    public double velocityFunctionX(double s, double time) {
        return super.velocityFunctionX(s, time);
    }

    @Override
    public double velocityFunctionY(double s, double time) {
        return super.velocityFunctionY(s, time);
    }

    @Override
    public double velocityFunctionTheta(double s, double time) {
        return super.velocityFunctionTheta(s, time);
    }

    @Override
    public double getS(double time) {
        return super.getS(time);
    }

    @Override
    public double getdS(double time) {
        return super.getdS(time);
    }
}