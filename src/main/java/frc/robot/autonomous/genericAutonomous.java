package frc.robot.autonomous;

import frc.robot.generic.GenericRobot;

public abstract class genericAutonomous {
    public int autonomousStep = 0;

    public void autonomousInit(GenericRobot robot){

    }

    public void autonomousPeriodic(GenericRobot robot){

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
    public double velocityFunctionX(double s){
        return 0;
    }
    public double velocityFunctionY(double s){
        return 0;
    }
    public double velocityFunctionTheta(double s){
        return 0;
    }
    public double vFX(double s, int autoStep) { return 0; }
    public double vFY(double s, int autoStep) { return 0; }

    public double getS(double time){
        return time;
    }

}