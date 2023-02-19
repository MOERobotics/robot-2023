package frc.robot.helpers;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonBox {
    static Joystick joystick;
    public ButtonBox(Joystick joystick) {
        this.joystick = joystick;
    }
    enum height {
        NONE,
        LOW,
        MIDDLE,
        HIGH
    }
    public static height getHeight() {
        if(!joystick.getRawButtonPressed(10) && !joystick.getRawButtonPressed(11)) {
            return height.MIDDLE;
        }
        if(!joystick.getRawButtonPressed(10) && joystick.getRawButtonPressed(11)) {
            return height.LOW;
        }
        if(joystick.getRawButtonPressed(10) && !joystick.getRawButtonPressed(11)) {
            return height.HIGH;
        }
        return height.NONE;
    }
    public static int getRow() {
        if(joystick.getRawButtonPressed(1)) {
            return 1;
        }
        if(joystick.getRawButtonPressed(2)) {
            return 2;
        }
        if(joystick.getRawButtonPressed(3)) {
            return 3;
        }
        if(joystick.getRawButtonPressed(4)) {
            return 4;
        }
        if(joystick.getRawButtonPressed(5)) {
            return 5;
        }
        if(joystick.getRawButtonPressed(6)) {
            return 6;
        }
        if(joystick.getRawButtonPressed(7)) {
            return 7;
        }
        if(joystick.getRawButtonPressed(8)) {
            return 8;
        }
        if(joystick.getRawButtonPressed(9)) {
            return 9;
        }
        return 0;
    }
}
