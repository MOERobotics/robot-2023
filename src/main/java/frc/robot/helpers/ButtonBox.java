package frc.robot.helpers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartDash;

public class ButtonBox {
    static Joystick joystick;
    public ButtonBox(Joystick joystick) {
        this.joystick = joystick;
    }
    public static int heightIndex = 0;
    public static int getHeight(Joystick joystick) {
        setHeight(joystick);
        return heightIndex;
    }
    public static boolean pressed = false;

    public static void setHeight(Joystick joystick) {
        SmartDashboard.putNumber("height Index", heightIndex);
        if(joystick.getRawButtonPressed(1) || joystick.getRawButton(4) || joystick.getRawButton(7)) {
            heightIndex = 0;
            pressed = true;
        }
        else if(joystick.getRawButtonPressed(2) || joystick.getRawButton(5) || joystick.getRawButton(8)) {
            heightIndex = 1;
            pressed = true;
        }
        else if(joystick.getRawButtonPressed(1) || joystick.getRawButton(4) || joystick.getRawButton(7)) {
            heightIndex = 2;
            pressed = true;
        }
    }
}
