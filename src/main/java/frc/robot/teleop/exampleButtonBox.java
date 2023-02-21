package frc.robot.teleop;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.ButtonBox;

import java.awt.*;

public class exampleButtonBox extends GenericTeleop {
    Joystick buttonbox = new Joystick(1);
    ButtonBox box = new ButtonBox(buttonbox);
    boolean go = false;

    @Override
    public void teleopPeriodic(GenericRobot robot) {
        ButtonBox.height height = Enum.valueOf(ButtonBox.height.class, String.valueOf(ButtonBox.getHeight()));
        int row = ButtonBox.getRow();
        // check if go pressed
        // if pressed, go = !go;
        if(go) {
            switch (row) {
                case 1:
                    //row 1
                    break;
                case 2:
                    //row 2
                    break;
                case 3:
                    //row 3
                    break;
                case 4:
                    //row 4
                    break;
                case 5:
                    //row 5
                    break;
                case 6:
                    //row 6
                    break;
                case 7:
                    //row 7
                    break;
                case 8:
                    //row 8
                    break;
                case 9:
                    //row 9
                    break;
                case 0:
                    break;
            }
            switch (height) {
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
}
