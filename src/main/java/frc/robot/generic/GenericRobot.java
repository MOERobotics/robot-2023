package frc.robot.generic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;

public abstract class GenericRobot {

    double oldLeftA = 0;
    double oldLeftB = 0;
    double oldRightA = 0;
    double oldRightB = 0;


    double offsetLeftA, offsetLeftB, offsetRightA, offsetRightB;

    double[] startDists;
    double[] startPivots;
    double startHeading;

///////////////////////////////////////////////////////////////////////////////////////////////////////Helpful Swerve commands
    public void swerve(SwerveModuleState frontLeft, SwerveModuleState frontRight,
                               SwerveModuleState backLeft, SwerveModuleState backRight){
        setLeftDriveARPM(frontLeft.speedMetersPerSecond);
        setPivotLeftMotorA(frontLeft.angle.getDegrees());

        setRightDriveARPM(frontRight.speedMetersPerSecond);
        setPivotRightMotorA(frontRight.angle.getDegrees());

        setLeftDriveBRPM(backLeft.speedMetersPerSecond);
        setPivotLeftMotorB(backLeft.angle.getDegrees());

        setRightDriveBRPM(backRight.speedMetersPerSecond);
        setPivotRightMotorB(backRight.angle.getDegrees());
    }

    public void stopSwerve(double oldLA, double oldRA, double oldLB, double oldRB){
        setLeftDriveARPM(0);
        setRightDriveARPM(0);
        setLeftDriveBRPM(0);
        setRightDriveBRPM(0);

        setPivotLeftMotorA(oldLA);
        setPivotLeftMotorB(oldLB);
        setPivotRightMotorA(oldRA);
        setPivotRightMotorB(oldRB);
    }

    public SwerveModuleState optimizeSwervePivots(SwerveModuleState desiredState, Rotation2d currentAngle){
            var delta = desiredState.angle.minus(currentAngle);
            if (Math.abs(delta.getDegrees()) > 150.0) {
                return new SwerveModuleState(
                        -desiredState.speedMetersPerSecond,
                        desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
            } else {
                return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
            }
    }
    public void SwerveAutoReset(){}

    public void SwerveControllerCommand(Trajectory trajectory, Pose2d pose, SwerveDriveKinematics kinematics, PIDController xController,
                                                PIDController yController, PIDController thetaController){}

    public Pose2d getPose(double startHeading, double currHeading, double[] startDistances, double[] startPivots, Pose2d startPose){
        return null;
    }

    public double deadzone(double value, double zone){
        if (Math.abs(value) < zone){
            value = 0;
        }
        else{
            if (value < 0) value = (value + zone)/(1 - zone);
            if (value > 0) value = (value - zone)/(1 - zone);
        }
        return value;
    }
    public double getMaxMeterPerSec(){
        return 0;
    }
    public double getMaxRadPerSec(){
        return 0;
    }
    public SwerveDriveKinematics kinematics(){
        return null;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////// NavX Commands
    public double getYaw() {
        return 0;
    }
    public double getRoll() {
        return 0;
    }
    public double getPitch() {
        return 0;
    }
    public void resetAttitude(){

    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Drive Motor Commands

    ////////////////////////////////////////////////////////// Encoders


    public double encoderLeftADriveTicksPerInch() {
        return 1.0;
    }
    public double encoderLeftBDriveTicksPerInch() {
        return 1.0;
    }
    public double encoderRightADriveTicksPerInch() {
        return 1.0;
    }
    public double encoderRightBDriveTicksPerInch() {
        return 1.0;
    }
    public double encoderTicksLeftDriveA() {
        return 0.0;
    }
    public double encoderTicksLeftDriveB() {
        return 0.0;
    }
    public double encoderTicksRightDriveA() {
        return 0.0;
    }
    public double encoderTicksRightDriveB() {
        return 0.0;
    }
    public double getDriveDistanceInchesLeftA() {
        return encoderTicksLeftDriveA()/encoderLeftADriveTicksPerInch();
    }
    public double getDriveDistanceInchesLeftB() {
        return encoderTicksLeftDriveB()/encoderLeftBDriveTicksPerInch();
    }
    public double getDriveDistanceInchesRightA() {
        return encoderTicksRightDriveA()/encoderRightADriveTicksPerInch();
    }
    public double getDriveDistanceInchesRightB() {
        return encoderTicksRightDriveB()/encoderRightBDriveTicksPerInch();
    }

    ////////////////////////////////////////////////////////// Motor Velocity
    public void setLeftDriveAPowerPercentage(double power) {

    }
    public void setLeftDriveBPowerPercentage(double power) {

    }
    public void setRightDriveAPowerPercentage(double power) {

    }
    public void setRightDriveBPowerPercentage(double power) {

    }
    public void setLeftDriveARPM(double rpm) {

    }
    public void setLeftDriveBRPM(double rpm) {

    }
    public void setRightDriveARPM(double rpm) {

    }
    public void setRightDriveBRPM(double rpm) {

    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Pivot Motor Commands

    ///////////////////////////////////////////////////////////////////////////drive motor

    public void resetPIDPivot(){

    }


    ////////////////////////////////////////////////////////// Encoders

    public void setOffsetLeftA(){}
    public void setOffsetLeftB(){}
    public void setOffsetRightA(){}
    public void setOffsetRightB(){}

    public double rawEncoderLeftA(){
        return 0;
    }
    public double rawEncoderLeftB(){
        return 0;
    }
    public double rawEncoderRightA(){
        return 0;
    }
    public double rawEncoderRightB(){
        return 0;
    }

    public double getPivotLeftMotorA() {
        return 0.0;
    }
    public double getPivotLeftMotorB() {
        return 0.0;
    }
    public double getPivotRightMotorA() {
        return 0.0;
    }
    public double getPivotRightMotorB() {
        return 0.0;
    }

    ////////////////////////////////////////////////////////// Motor Velocity

    public void setPivotLeftMotorA(double Pivot) {

    }
    public void setPivotLeftMotorB(double Pivot) {

    }
    public void setPivotRightMotorA(double Pivot) {

    }
    public void setPivotRightMotorB(double Pivot) {

    }
    ////////////////////////////////////////////////////////////////////Drive function
    public void setDrive(double xspd, double yspd, double turnspd){

    }

    static final Pose2d defaultPose = new Pose2d(0,0,new Rotation2d(0));
    public Pose2d getPose() {
        return defaultPose;
    }

    
    public void resetStartHeading() {
        startHeading = getYaw();
    }

    public void resetStartDists() {
        startDists = new double[] {getDriveDistanceInchesLeftA(), getDriveDistanceInchesRightA(),
                getDriveDistanceInchesLeftB(),getDriveDistanceInchesRightB()};
    }

    public void resetStartPivots() {
        startPivots = new double[] {getPivotLeftMotorA(), getPivotRightMotorA(),
                getPivotLeftMotorB(), getPivotRightMotorB()};
    }
}