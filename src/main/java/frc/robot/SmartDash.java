package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class SmartDash {

    public static void Dashboard(GenericRobot robot){
        SmartDashboard.putNumber("armPotDegrees", robot.getPotDegrees());
        SmartDashboard.putNumber("armInches", robot.getArmPosition());

        SmartDashboard.putNumber("yaw", robot.getYaw());
        SmartDashboard.putNumber("leftApivot", robot.getPivotLeftMotorA());
        SmartDashboard.putNumber("leftBpivot", robot.getPivotLeftMotorB());
        SmartDashboard.putNumber("rightApivot", robot.getPivotRightMotorA());
        SmartDashboard.putNumber("rightBpivot", robot.getPivotRightMotorB());

        SmartDashboard.putNumber("leftApivotRaw", robot.rawEncoderLeftA());
        SmartDashboard.putNumber("leftBpivotRaw", robot.rawEncoderLeftB());
        SmartDashboard.putNumber("rightApivotRaw", robot.rawEncoderRightA());
        SmartDashboard.putNumber("rightBpivotRaw", robot.rawEncoderRightB());
        SmartDashboard.putNumber("pitch", robot.getPitch());
        SmartDashboard.putNumber("roll", robot.getRoll());
        SmartDashboard.putNumber("pigeonYaw", robot.getPigeonYaw());
        SmartDashboard.putNumber("pigeonPitch", robot.getPigeonPitch());
        SmartDashboard.putNumber("pigeonRoll", robot.getPigeonRoll());
        SmartDashboard.putNumber("pigeonCompass", robot.getAbsoluteCompassHeadingPigeon());

        SmartDashboard.putBoolean("Red Robot", robot.getRed());

        SmartDashboard.putNumber("armPosition", robot.getArmPosition());
        SmartDashboard.putBoolean("cargoInCollect", robot.cargoInCollector());
        SmartDashboard.putBoolean("cargoDetected", robot.cargoDetected());
        SmartDashboard.putBoolean("armHitLimit", robot.armHitLimit());
        SmartDashboard.putNumber("leftACurrent", robot.getLeftACurrent());
        SmartDashboard.putNumber("leftBCurrent", robot.getLeftBCurrent());
        SmartDashboard.putNumber("rightACurrent", robot.getRightACurrent());
        SmartDashboard.putNumber("rightBCurrent", robot.getRightBCurrent());
        SmartDashboard.putBoolean("floorSensorLeft", robot.getLeftFloorSensor());
        SmartDashboard.putBoolean("floorSensorRight", robot.getRightFloorSensor());

        SmartDashboard.putNumber("pigeonBoundYaw", robot.getPigeonBoundedYaw(robot.getPigeonYaw()));
        SmartDashboard.putNumber("TimeOfFlight Range", robot.getTOFDistance());

        robot.getDriveDistanceInchesLeftA();
        robot.getDriveDistanceInchesLeftB();
        robot.getDriveDistanceInchesRightB();
        robot.getDriveDistanceInchesRightA();
    }
}
