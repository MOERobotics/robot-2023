package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.vision.MoeNetVision;

public class AbsolutePosition extends genericAutonomous {

    MoeNetVision mnv;

    public AbsolutePosition(MoeNetVision mnv) {
        this.mnv = mnv;
    }

    Pose2d target = new Pose2d(14, 4.4, new Rotation2d(0));
    double desiredInPerSec = 40;
    double ds = desiredInPerSec;

    PIDController yPID = new PIDController(1e-5, 0, 0);

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("autostep", autonomousStep);
        Pose2d currPose = mnv.getPose().toPose2d();

        double xspd = 0, yspd = 0, turnspd = 0;

        switch (autonomousStep) {
            case 0:
                robot.resetStartDists();
                robot.resetStartPivots();
                robot.resetStartHeading();
                xspd = yspd = turnspd = 0;
                yPID.reset();
                autonomousStep++;
                break;
            case 1:
                xspd = 0;
                double error = Math.abs(currPose.getY() - target.getY());
                yspd = yPID.calculate(error);

                if(yspd>8)yspd=8;
                if(yspd<-8)yspd=-8;

                if (error < .04) {
                    autonomousStep++;
                }
                break;
            case 2:
                xspd = 0;
                yspd = 0;
                turnspd = 0;
        }
        robot.setDrive(xspd, yspd, turnspd);
    }
}
