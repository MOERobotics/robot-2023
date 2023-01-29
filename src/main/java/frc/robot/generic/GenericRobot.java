package frc.robot.generic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface GenericRobot {
///////////////////////////////////////////////////////////////////////////////////////////////////////Helpful Swerve commands
    public default void swerve(SwerveModuleState frontLeft, SwerveModuleState frontRight,
                               SwerveModuleState backLeft, SwerveModuleState backRight){
        setLeftDriveARPM(frontLeft.speedMetersPerSecond*convertInchpsToRPM());
        setPivotLeftMotorA(frontLeft.angle.getDegrees());

        setRightDriveARPM(frontRight.speedMetersPerSecond*convertInchpsToRPM());
        setPivotRightMotorA(frontRight.angle.getDegrees());

        setLeftDriveBRPM(backLeft.speedMetersPerSecond*convertInchpsToRPM());
        setPivotLeftMotorB(backLeft.angle.getDegrees());

        setRightDriveBRPM(backRight.speedMetersPerSecond*convertInchpsToRPM());
        setPivotRightMotorB(backRight.angle.getDegrees());
    }

    public default void stopSwerve(double oldLA, double oldRA, double oldLB, double oldRB){
        setLeftDriveARPM(0);
        setRightDriveARPM(0);
        setLeftDriveBRPM(0);
        setRightDriveBRPM(0);

        setPivotLeftMotorA(oldLA);
        setPivotLeftMotorB(oldLB);
        setPivotRightMotorA(oldRA);
        setPivotRightMotorB(oldRB);
    }

    public default SwerveModuleState optimizeSwervePivots(SwerveModuleState desiredState, Rotation2d currentAngle){
            var delta = desiredState.angle.minus(currentAngle);
            if (Math.abs(delta.getDegrees()) > 150.0) {
                return new SwerveModuleState(
                        -desiredState.speedMetersPerSecond,
                        desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
            } else {
                return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
            }
    }
    public default void SwerveAutoReset(){}

    public default void SwerveControllerCommand(Trajectory trajectory, Pose2d pose, SwerveDriveKinematics kinematics, PIDController xController,
                                                PIDController yController, PIDController thetaController){}

    public default Pose2d getPose(double startHeading, double currHeading, double[] startDistances, double[] startPivots, Pose2d startPose){
        return null;
    }

    public default double deadzone(double value, double zone){
        if (Math.abs(value) < zone){
            value = 0;
        }
        else{
            if (value < 0) value = (value + zone)/(1 - zone);
            if (value > 0) value = (value - zone)/(1 - zone);
        }
        return value;
    }
    public default double getMaxInchesPerSecond(){
        return 0;
    }
    public default double getMaxRadPerSec(){
        return 0;
    }
    public default SwerveDriveKinematics kinematics(){
        return null;
    }
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
    public default void resetAttitude(){

    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Pigeon Commands

    public default double getPigeonYaw(){
        return 0;
    }

    public default double getPigeonRoll(){
        return 0;
    }

    public default double getPigeonPitch(){
        return 0;
    }

    public default double getAbsoluteCompassHeadingPigeon(){
        return 0;
    }

    public default void resetPigeon(){
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

    public default double convertInchpsToRPM(){
        return 0.0;
    }

    public default double getDriveDistanceInchesLeftA() {
        double val = encoderTicksLeftDriveA()/encoderLeftADriveTicksPerInch();
        SmartDashboard.putNumber("DriveInchesLeftA", val);
        return val;

    }
    public default double getDriveDistanceInchesLeftB() {
        double val = encoderTicksLeftDriveB()/encoderLeftBDriveTicksPerInch();
        SmartDashboard.putNumber("DriveInchesLeftB", val);
        return val;
    }
    public default double getDriveDistanceInchesRightA() {
        double val = encoderTicksRightDriveA()/encoderRightADriveTicksPerInch();
        SmartDashboard.putNumber("DriveInchesRightA", val);
        return val;
    }
    public default double getDriveDistanceInchesRightB() {
        double val = encoderTicksRightDriveB()/encoderRightBDriveTicksPerInch();
        SmartDashboard.putNumber("DriveInchesRightB", val);
        return val;
    }

    ////////////////////////////////////////////////////////// Motor Velocity
    public default void setLeftDriveAPowerPercentage(double power) {

    }
    public default void setLeftDriveBPowerPercentage(double power) {

    }
    public default void setRightDriveAPowerPercentage(double power) {

    }
    public default void setRightDriveBPowerPercentage(double power) {

    }
    public default void setLeftDriveARPM(double rpm) {

    }
    public default void setLeftDriveBRPM(double rpm) {

    }
    public default void setRightDriveARPM(double rpm) {

    }
    public default void setRightDriveBRPM(double rpm) {

    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Pivot Motor Commands

    public default void resetPIDPivot(){

    }


    ////////////////////////////////////////////////////////// Encoders

    public default void setOffsetLeftA(){}
    public default void setOffsetLeftB(){}
    public default void setOffsetRightA(){}
    public default void setOffsetRightB(){}

    public default double rawEncoderLeftA(){
        return 0;
    }
    public default double rawEncoderLeftB(){
        return 0;
    }
    public default double rawEncoderRightA(){
        return 0;
    }
    public default double rawEncoderRightB(){
        return 0;
    }

    public default double getPivotLeftMotorA() {
        return 0.0;
    }
    public default double getPivotLeftMotorB() {
        return 0.0;
    }
    public default double getPivotRightMotorA() {
        return 0.0;
    }
    public default double getPivotRightMotorB() {
        return 0.0;
    }

    ////////////////////////////////////////////////////////// Motor Velocity

    public default void setPivotLeftMotorA(double Pivot) {

    }
    public default void setPivotLeftMotorB(double Pivot) {

    }
    public default void setPivotRightMotorA(double Pivot) {

    }
    public default void setPivotRightMotorB(double Pivot) {

    }

}