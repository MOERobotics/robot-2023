package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.A1CDock;
import frc.robot.autonomous.genericAutonomous;

public class AutoSelector{
    public static final genericAutonomous
            A1CDock = new A1CDock();
    static Joystick xbox = new Joystick(1);
    static genericAutonomous autonomous;
    public static genericAutonomous select(){
        SmartDashboard.putString("autonomous", autonomous.getClass().getName());
        if (xbox.getRawButtonPressed(1)){
            autonomous = A1CDock;
        }
        return autonomous;
    }
}
