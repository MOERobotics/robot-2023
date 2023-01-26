package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generic.GenericRobot;

public class Drive5Feet extends genericAutonomous {
    Pose2d current_position;
    int autonomousStep = 0;

    double start_distance;
    double default_power;
    double start_time;

    public void autonomousInit(GenericRobot robot){
        autonomousStep = 0;
        start_time = System.currentTimeMillis();
        //TODO: add pid stuff
    }

    public void autonomousPeriodic(GenericRobot robot){
        switch(autonomousStep){
            case 0: //reset
                if(System.currentTimeMillis() - start_time > 100){
                    autonomousStep += 1;
                }
                break;
            case 1: //drive forward 5ft
                current_position = robot.getPose();
                //TODO: add getPose stuff
                autonomousStep += 1;
                break;
            case 2: //stop

                break;
        }
    }
}
