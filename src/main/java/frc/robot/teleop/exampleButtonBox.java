package frc.robot.teleop;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.ButtonBox;

public class exampleButtonBox extends GenericTeleop {
    Joystick buttonbox = new Joystick(1);
    ButtonBox box = new ButtonBox(buttonbox);
    boolean go = false;

    @Override
    public void teleopPeriodic(GenericRobot robot) {
    }
}
