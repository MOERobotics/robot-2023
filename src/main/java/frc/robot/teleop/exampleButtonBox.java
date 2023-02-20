package frc.robot.teleop;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.ButtonBox;

import java.awt.*;

public class exampleButtonBox extends GenericTeleop{
    Joystick buttonbox = new Joystick(1);
    ButtonBox box = new ButtonBox(buttonbox);
    @Override
    public void teleopPeriodic(GenericRobot robot) {
        ButtonBox.height height = Enum.valueOf(ButtonBox.height.class, String.valueOf(box.getHeight()));
        switch(height) {
            case LOW:
                //Low
                break;
            case MIDDLE:
                //Middle
                break;
            case HIGH:
                //High
                break;
            case NONE:
                break;
        }
    }
}
