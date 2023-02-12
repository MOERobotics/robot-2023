package frc.robot.teleop;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.generic.GenericRobot;

public class ArmCode extends GenericTeleop{

    Joystick xbox = new Joystick(1);
    double collectorRPM = 0;
    double armPower = 0;
    boolean liftTopRoller = false;
    double topNodeConeHeight = 46;
    double midNodeConeHeight = 34;
    double topCubeHeight = 35.5;
    double midCubeHeight = 23.5;
    double xVal = 53.64+14;
    double fieldLength = 325.25*2;
    double[] nodeYPositions = {19.5,45.1, 66.4, 89.3, 111.54, 133.43, 156.26, 176.91, 201.02};

    //////////////////////////////////////Ideally have a button box and press where you want a cube/cone depoed.

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

        armPower = xbox.getRawAxis(1);
        if (robot.getArmPosition() > topNodeConeHeight || robot.getArmPosition() < 0){
            armPower = 0;
        }

        ///////////////////////////////////////////////////////////////////////////Power setters
        robot.collect(collectorRPM);
        robot.raiseTopRoller(liftTopRoller);
        robot.moveArm(armPower);

    }
}
