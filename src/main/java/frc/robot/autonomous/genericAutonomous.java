package frc.robot.autonomous;

import frc.robot.generic.GenericRobot;

public abstract class genericAutonomous {
    public int autonomousStep = 0;

    public void autonomousInit(GenericRobot robot){

    }

    public void autonomousPeriodic(GenericRobot robot){

    }

    public abstract void autonomousInit();

    public abstract void autonomousPeriodic();
}