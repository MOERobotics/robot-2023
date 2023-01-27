package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.generic.GenericRobot;

public abstract class genericAutonomous {
    public int autonomousStep = 0;

    public void autonomousInit(GenericRobot robot){

    }

    public void autonomousPeriodic(GenericRobot robot){
    }

    public void trajectoryCalc(GenericRobot robot, Pose2d origin, Pose2d dest) {

    }
    public Pose2d diff(GenericRobot robot, Pose2d one, Pose2d two) {
        return new Pose2d();
    }
}