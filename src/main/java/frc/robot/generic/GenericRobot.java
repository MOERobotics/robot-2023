package frc.robot.generic;

public interface GenericRobot {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////// NavX Commands
    public default double getYaw() {
        return 0;
    }
    public default double getRoll() {
        return 0;
    }
    public default double getPitch() {
        return 0;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Drive Motor Commands

    ////////////////////////////////////////////////////////// Encoders
    public default double encoderLeftADriveTicksPerInch() {
        return 1.0;
    }
    public default double encoderLeftBDriveTicksPerInch() {
        return 1.0;
    }
    public default double encoderRightADriveTicksPerInch() {
        return 1.0;
    }
    public default double encoderRightBDriveTicksPerInch() {
        return 1.0;
    }
    public default double encoderTicksLeftDriveA() {
        return 0.0;
    }
    public default double encoderTicksLeftDriveB() {
        return 0.0;
    }
    public default double encoderTicksRightDriveA() {
        return 0.0;
    }
    public default double encoderTicksRightDriveB() {
        return 0.0;
    }
    public default double getDriveDistanceInchesLeftA() {
        return encoderTicksLeftDriveA()/encoderLeftADriveTicksPerInch();
    }
    public default double getDriveDistanceInchesLeftB() {
        return encoderTicksLeftDriveB()/encoderLeftBDriveTicksPerInch();
    }
    public default double getDriveDistanceInchesRightA() {
        return encoderTicksRightDriveA()/encoderRightADriveTicksPerInch();
    }
    public default double getDriveDistanceInchesRightB() {
        return encoderTicksRightDriveB()/encoderRightBDriveTicksPerInch();
    }

    ////////////////////////////////////////////////////////// Motor Velocity
    public default void setLeftDriveAPowerPercentage() {

    }
    public default void setLeftDriveBPowerPercentage() {

    }
    public default void setRightDriveAPowerPercentage() {

    }
    public default void setRightDriveBPowerPercentage() {

    }
    public default void setLeftDriveARPM() {

    }
    public default void setLeftDriveBRPM() {

    }
    public default void setRightDriveARPM() {

    }
    public default void setRightDriveBRPM() {

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Pivot Motor Commands

    ////////////////////////////////////////////////////////// Encoders
    public default double encoderLeftAPivotTicksPerInch() {
        return 1.0;
    }
    public default double encoderLeftBPivotTicksPerInch() {
        return 1.0;
    }
    public default double encoderRightAPivotTicksPerInch() {
        return 1.0;
    }
    public default double encoderRightBPivotTicksPerInch() {
        return 1.0;
    }
    public default double encoderTicksLeftPivotA() {
        return 0.0;
    }
    public default double encoderTicksLeftPivotB() {
        return 0.0;
    }
    public default double encoderTicksRightPivotA() {
        return 0.0;
    }
    public default double encoderTicksRightPivotB() {
        return 0.0;
    }
    public default double getPivotDistanceInchesLeftA() {
        return encoderTicksLeftPivotA()/encoderLeftAPivotTicksPerInch();
    }
    public default double getPivotDistanceInchesLeftB() {
        return encoderTicksLeftPivotB()/encoderLeftBPivotTicksPerInch();
    }
    public default double getPivotDistanceInchesRightA() {
        return encoderTicksRightPivotA()/encoderRightAPivotTicksPerInch();
    }
    public default double getPivotDistanceInchesRightB() {
        return encoderTicksRightPivotB()/encoderRightBPivotTicksPerInch();
    }

    ////////////////////////////////////////////////////////// Motor Velocity
    public default void setPivotLeftMotorA(double Pivot) {

    }
    public default double getPivotLeftMotorA() {
        return 0.0;
    }
    public default void setPivotLeftMotorB(double Pivot) {

    }
    public default double getPivotLeftMotorB() {
        return 0.0;
    }
    public default void setPivotRighttMotorA(double Pivot) {

    }
    public default double getPivotRightMotorA() {
        return 0.0;
    }
    public default void setPivotRightMotorB(double Pivot) {

    }
    public default double getPivotRightMotorB() {
        return 0.0;
    }
}