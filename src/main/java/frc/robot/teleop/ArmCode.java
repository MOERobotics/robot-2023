package frc.robot.teleop;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.generic.GenericRobot;

public class ArmCode extends GenericTeleop{

    Joystick xbox = new Joystick(1);
    double collectorRPM = 0;
    double armPower = 0;
    boolean liftTopRoller = false;


    @Override
    public void teleopInit(GenericRobot robot) {

    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {
        // Bumpers left 5, right 6

        if (xbox.getRawButton(5)){ //move roller up and down
            liftTopRoller = true;
        }
        else if (xbox.getRawButton(6)){
            liftTopRoller = false;
        }

        // 2 is b, 3 is x

        if (xbox.getRawButton(3)){ //collect in and out
            collectorRPM = 1000;
        }
        else if (xbox.getRawButton(2)){
            collectorRPM = -1000;
        }
        else{
            collectorRPM = 0;
        }

        armPower = robot.deadzone(xbox.getRawAxis(1), .2);

        ///////////////////////////////////////////////////////////////////////////Power setters
        robot.collect(collectorRPM);
        robot.raiseTopRoller(liftTopRoller);
        robot.moveArm(armPower);

    }
}
