package frc.robot.generic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class GenericRobot {

    double oldLeftA = 0;
    double oldLeftB = 0;
    double oldRightA = 0;
    double oldRightB = 0;


    double offsetLeftA, offsetLeftB, offsetRightA, offsetRightB;

    double[] startDists;
    double[] startPivots;
    double startHeading;

    boolean Red;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////Helpful Swerve commands
    public void swerve(SwerveModuleState frontLeft, SwerveModuleState frontRight,
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
            if (Math.abs(delta.getDegrees()) > 90.0) {
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
    public double getMaxInchesPerSecond(){
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Pigeon Commands

    public double getPigeonYaw(){
        return 0;
    }

    public double getPigeonRoll(){
        return 0;
    }

    public double getPigeonPitch(){
        return 0;
    }

    public double getAbsoluteCompassHeadingPigeon(){
        return 0;
    }

    public void resetPigeon(){
    }

    public void setPigeonYaw(double startYaw){}

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

    public double convertInchpsToRPM(){
        return 0.0;
    }

    public double getDriveDistanceInchesLeftA() {
        double val = encoderTicksLeftDriveA()/encoderLeftADriveTicksPerInch();
        SmartDashboard.putNumber("DriveInchesLeftA", val);
        return val;

    }
    public double getDriveDistanceInchesLeftB() {
        double val = encoderTicksLeftDriveB()/encoderLeftBDriveTicksPerInch();
        SmartDashboard.putNumber("DriveInchesLeftB", val);
        return val;
    }
    public double getDriveDistanceInchesRightA() {
        double val = encoderTicksRightDriveA()/encoderRightADriveTicksPerInch();
        SmartDashboard.putNumber("DriveInchesRightA", val);
        return val;
    }
    public double getDriveDistanceInchesRightB() {
        double val = encoderTicksRightDriveB()/encoderRightBDriveTicksPerInch();
        SmartDashboard.putNumber("DriveInchesRightB", val);
        return val;
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

    public double getLeftACurrent(){return 0;}
    public double getRightACurrent(){return 0;}
    public double getLeftBCurrent(){return 0;}
    public double getRightBCurrent(){return 0;}

    public double wallHit(){
        return 0;
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

    public void setDrive(double xspd, double yspd, double turnspd, boolean auto, boolean fieldCentric){

    }

    public void setDrive(double xspd, double yspd, double turnspd, boolean auto){}

    static final Pose2d defaultPose = new Pose2d(0,0,new Rotation2d(0));
    public Pose2d getPose() {
        return defaultPose;
    }

    public void setPose(Pose2d startPose){}
    public void setPose(){}


    public void resetStartHeading() {
        startHeading = getPigeonYaw();
    }

    public void resetStartDists() {
        startDists = new double[] {getDriveDistanceInchesLeftA(), getDriveDistanceInchesRightA(),
                getDriveDistanceInchesLeftB(),getDriveDistanceInchesRightB()};
    }

    public void resetStartPivots() {
        startPivots = new double[] {getPivotLeftMotorA(), getPivotRightMotorA(),
                getPivotLeftMotorB(), getPivotRightMotorB()};
    }

    /////////////////////////////////////////////////////////////////////////////////////////Collector Code

    public void setBottomRollerPower(double power){}
    public void setTopRollerPower(double power){}

    public void collect(double rpm, boolean autoMode){}
    public void collect(double rpm){
        this.collect(rpm, false);
    }
    public void setBottomRollerRPM(double rpm){}
    public void setTopRollerRPM(double rpm){}

    public void raiseTopRoller(boolean up){}

    public double getTopRollerPosition(){return 0;}

    public boolean cargoInCollector(){return false;}

    public boolean cargoDetected(){return false;}
    public boolean armHitLimit(){return false;}

    ////////////////////////////////////////////////////////////////////////////////////Arm Code

    public void rightArmPower(double power){}
    public void leftArmPower(double power){}
    public void moveArm(double power){}
    public double getArmPosition(){return 0;}

    public double getPotDegrees(){return 0;}
    public double getPotRadians(){return 0;}

    public void stackCargo(double zPos){

    }
    public void holdArmPosition(double pos){

    }

    public void liftArm(){

    }

    public void dropArm(){}
    ///////////////////////////////////////////////////////////////////////////////////Gripper Code
    public void openGripper(boolean open){}

    public boolean gripperIsOpen(){return false;}

    ///////////////////////////////////////////////////////////////////////////////////Tape Sensors

    public boolean getLeftASensor(){return false;}
    public boolean getLeftBSensor(){return false;}
    public boolean getRightASensor(){return false;}
    public boolean getRightBSensor(){return false;}

    public boolean getRed(){return Red;}

    public void setRed(boolean redStatus){Red = redStatus;}

    ///////////////////////////////////////////////////////////Light Sensors
    public boolean getLeftLightSensor(){return false;}

    public boolean getRightLightSensor(){return false;}

    ////////////////////////////////////////////////////////////solenoid lights

    public void setLightsOn(boolean on){
    }

}