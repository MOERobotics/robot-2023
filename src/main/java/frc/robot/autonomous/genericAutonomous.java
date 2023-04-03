package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public abstract class genericAutonomous {
    public int autonomousStep = 0;

    public void autonomousInit(GenericRobot robot){

    }

    public void autonomousPeriodic(GenericRobot robot){
        SmartDashboard.putNumber("autoStep", autonomousStep);
    }

    public double positionFunctionX(double s){
        return 0;
    }
    public double positionFunctionY(double s){
        return 0;
    }
    public double positionFunctionTheta(double s){
        return 0;
    }
    public double velocityFunctionX(double s, double time){
        return 0;
    }
    public double velocityFunctionY(double s, double time){
        return 0;
    }
    public double velocityFunctionTheta(double s, double time){
        return 0;
    }
    public double getS(double time){
        return time;
    }

    public double getdS(double time){return 0;}

}