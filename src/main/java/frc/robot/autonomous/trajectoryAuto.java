package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class trajectoryAuto extends genericAutonomous{
    Trajectory myTrajectory;
    Pose2d startPose, currPose;
    double startYaw;
    double[] startDist;
    double[] startPivot;
    PIDController xControl = new PIDController(0.15,0,0);
    PIDController yControl = new PIDController(0,0,0);
    PIDController radControl = new PIDController(0.7,0.01,0);

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        myTrajectory = createTrajectory.generateTrajectory(robot);
        radControl.enableContinuousInput(-Math.PI, Math.PI);
        robot.setOffsetLeftA();
        robot.setOffsetLeftB();
        robot.setOffsetRightA();
        robot.setOffsetRightB();

        robot.resetAttitude();
        robot.resetPIDPivot();
        xControl.reset();
        yControl.reset();
        radControl.reset();

        startYaw = robot.getYaw();

        startDist = new double[] {robot.getDriveDistanceInchesLeftA(), robot.getDriveDistanceInchesRightA(),
                robot.getDriveDistanceInchesLeftB(),robot.getDriveDistanceInchesRightB()};

        startPivot = new double[] {robot.getPivotLeftMotorA(), robot.getPivotRightMotorA(),
                robot.getPivotLeftMotorB(), robot.getPivotRightMotorB()};
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        switch(autonomousStep){
            case 0:
                robot.SwerveAutoReset();
                robot.resetStartHeading();
                robot.resetStartDists();
                robot.resetStartPivots();
                startYaw = robot.getYaw();
                startDist = new double[] {robot.getDriveDistanceInchesLeftA(), robot.getDriveDistanceInchesRightA(),
                robot.getDriveDistanceInchesLeftB(), robot.getDriveDistanceInchesRightB()};
                startPivot = new double[] {robot.getPivotLeftMotorA(), robot.getPivotRightMotorA(),
                robot.getDriveDistanceInchesLeftB(), robot.getDriveDistanceInchesRightB()};
                autonomousStep += 1;
                break;
            case 1:
                currPose = robot.getPose();
                robot.SwerveControllerCommand(myTrajectory, currPose, robot.kinematics(), xControl, yControl,
                        radControl);
                /*if (currPose.getX() >= 48){
                    robot.stopSwerve(0,0,0,0);
                    autonomousStep += 1;
                }*/
                break;
            case 2:
                robot.stopSwerve(0,0,0,0);
                break;
        }
    }
}
