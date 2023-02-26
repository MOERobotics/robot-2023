package frc.robot.helpers;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonBox {
    static Joystick joystick;
    public ButtonBox(Joystick joystick) {
        this.joystick = joystick;
    }
    public enum height {
        NONE,
        LOW,
        MIDDLE,
        HIGH
    }
    static height currentHeight;
    static int currentRow;
    public static height getHeight() {
        return currentHeight;
        /*if(!joystick.getRawButtonPressed(10) && !joystick.getRawButtonPressed(11)) {
            return height.MIDDLE;
        }
        if(!joystick.getRawButtonPressed(10) && joystick.getRawButtonPressed(11)) {
            return height.LOW;
        }
        if(joystick.getRawButtonPressed(10) && !joystick.getRawButtonPressed(11)) {
            return height.HIGH;
        }
        return height.NONE;*/
    }
    public static int getRow() {
        return currentRow;
        /*if(joystick.getRawButtonPressed(1)) {
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

         */
    }
    /*public static void setRow() {
        if(joystick.getRawButtonPressed(1)) {
            currentRow = 1;
        }
        if(joystick.getRawButtonPressed(2)) {
            currentRow = 2;
        }
        if(joystick.getRawButtonPressed(3)) {
            currentRow = 3;
        }
        if(joystick.getRawButtonPressed(4)) {
            currentRow = 4;
        }
        if(joystick.getRawButtonPressed(5)) {
            currentRow = 5;
        }
        if(joystick.getRawButtonPressed(6)) {
            currentRow = 6;
        }
        if(joystick.getRawButtonPressed(7)) {
            currentRow = 7;
        }
        if(joystick.getRawButtonPressed(8)) {
            currentRow = 8;
        }
        if(joystick.getRawButtonPressed(9)) {
            currentRow = 9;
        }
        currentRow = 0;
    }
    public static void setHeight() {
        if(!joystick.getRawButtonPressed(10) && !joystick.getRawButtonPressed(11)) {
            currentHeight = height.MIDDLE;
        }
        if(!joystick.getRawButtonPressed(10) && joystick.getRawButtonPressed(11)) {
            currentHeight = height.LOW;
        }
        if(joystick.getRawButtonPressed(10) && !joystick.getRawButtonPressed(11)) {
            currentHeight = height.HIGH;
        }
        currentHeight = height.NONE;
    }*/
    public static void set() {
        if(joystick.getRawButtonPressed(1)) {
            currentRow = 1;
        }
        if(joystick.getRawButtonPressed(2)) {
            currentRow = 2;
        }
        if(joystick.getRawButtonPressed(3)) {
            currentRow = 3;
        }
        if(joystick.getRawButtonPressed(4)) {
            currentRow = 4;
        }
        if(joystick.getRawButtonPressed(5)) {
            currentRow = 5;
        }
        if(joystick.getRawButtonPressed(6)) {
            currentRow = 6;
        }
        if(joystick.getRawButtonPressed(7)) {
            currentRow = 7;
        }
        if(joystick.getRawButtonPressed(8)) {
            currentRow = 8;
        }
        if(joystick.getRawButtonPressed(9)) {
            currentRow = 9;
        }
        if(!joystick.getRawButtonPressed(10) && !joystick.getRawButtonPressed(11)) {
            currentHeight = height.MIDDLE;
        }
        if(!joystick.getRawButtonPressed(10) && joystick.getRawButtonPressed(11)) {
            currentHeight = height.LOW;
        }
        if(joystick.getRawButtonPressed(10) && !joystick.getRawButtonPressed(11)) {
            currentHeight = height.HIGH;
        }
    }
}
