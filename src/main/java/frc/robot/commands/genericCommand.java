package frc.robot.commands;

import frc.robot.generic.GenericRobot;

public abstract class genericCommand {


    public void init(){}

    public void periodic(){}

    public abstract void init(GenericRobot robot);

    public abstract void periodic(GenericRobot robot);
}