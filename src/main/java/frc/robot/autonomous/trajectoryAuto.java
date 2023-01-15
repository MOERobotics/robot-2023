package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.generic.GenericRobot;

public class trajectoryAuto extends genericAutonomous{
    Trajectory myTrajectory;
    Pose2d startPose;
    double startYaw;
    double[] startDist;
    double[] startPivot;
    PIDController xControl = new PIDController(0,0,0);
    PIDController yControl = new PIDController(0,0,0);
    ProfiledPIDController radControl = new ProfiledPIDController(0,0,0,
            new TrapezoidProfile.Constraints(10, 20));

    @Override
    public void autonomousInit(GenericRobot robot) {
        myTrajectory = createTrajectory.generateTrajectory();
        radControl.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        switch(autonomousStep){
            case 0:
                robot.SwerveAutoReset();
                startYaw = robot.getYaw();
                startDist = new double[] {robot.getDriveDistanceInchesLeftA(), robot.getDriveDistanceInchesRightA(),
                robot.getDriveDistanceInchesLeftB(), robot.getDriveDistanceInchesRightB()};
                startPivot = new double[] {robot.getPivotLeftMotorA(), robot.getPivotRightMotorA(),
                robot.getDriveDistanceInchesLeftB(), robot.getDriveDistanceInchesRightB()};
                startPose = robot.getPose(robot.getYaw(), robot.getYaw(), startDist, startPivot,
                        new Pose2d(0,0, Rotation2d.fromDegrees(0)));
                autonomousStep += 1;
                break;
            case 1:
                Pose2d currPose = robot.getPose(startYaw, robot.getYaw(),startDist, startPivot, startPose);
                robot.SwerveControllerCommand(myTrajectory, currPose, robot.kinematics(), xControl, yControl,
                        radControl);
                if (currPose == myTrajectory.getStates().get(myTrajectory.getStates().size() - 1).poseMeters){
                    autonomousStep += 1;
                }
                break;
            case 2:
                robot.stopSwerve();
                break;
        }
    }
}
