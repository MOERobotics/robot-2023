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
        if (xbox.getRawButton(5)){ //move roller up and down
            liftTopRoller = true;
        }
        else if (xbox.getRawButton(6)){
            liftTopRoller = false;
        }

        if (xbox.getRawButton(3)){ //collect in and out
            collectorRPM = 1000;
        }
        else if (xbox.getRawButton(2)){
            collectorRPM = -1000;
        }
        else{
            collectorRPM = 0;
        }

        armPower = xbox.getRawAxis(1);

        ///////////////////////////////////////////////////////////////////////////Power setters
        robot.collect(collectorRPM);
        robot.raiseTopRoller(liftTopRoller);
        robot.moveArm(armPower);

    }
}
