package frc.robot.teleop;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.generic.GenericRobot;

public class ArmCode extends GenericTeleop{

    Joystick xbox = new Joystick(1);


    @Override
    public void teleopInit(GenericRobot robot) {

    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {
        if (xbox.getRawButton(5)){
            robot.setTopRollerPosPower(.5);
        }
        else if (xbox.getRawButton(6)){
            robot.setTopRollerPosPower(-.5);
        }
        else{
            robot.setTopRollerPosPower(0);
        }

        if (xbox.getRawButton(3)){

        }
    }
}
