package frc.robot.generic;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import com.revrobotics.AnalogInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static com.revrobotics.SparkMaxAnalogSensor.Mode.kAbsolute;
import static com.revrobotics.SparkMaxAnalogSensor.Mode.kRelative;

public class TherMOEDynamic extends GenericRobot{
    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);
    public final double MAX_ARM_HEIGHT = 51.75;
    public final double armRadius = 44;
    public final double MIN_ARM_HEIGHT = MAX_ARM_HEIGHT-armRadius-2;

    public final double lowestAngle = -4;
    public final double greatestAngle = 105;
    public final double shoulderCalib = 3.21;
    WPI_Pigeon2 pigeon = new WPI_Pigeon2(0);

///////////////////////////////////////////////////////////////////////////////////////swerve Motors and pivots
    CANSparkMax leftMotorA        = new CANSparkMax(19, kBrushless);
    CANSparkMax pivotLeftMotorA   = new CANSparkMax(18, kBrushless);

    CANSparkMax leftMotorB        = new CANSparkMax( 17, kBrushless);
    CANSparkMax pivotLeftMotorB   = new CANSparkMax(16, kBrushless);

    CANSparkMax rightMotorA       = new CANSparkMax(1, kBrushless);
    CANSparkMax pivotRightMotorA   = new CANSparkMax(20, kBrushless);

    CANSparkMax rightMotorB       = new CANSparkMax(13, kBrushless);
    CANSparkMax pivotRightMotorB   = new CANSparkMax(12, kBrushless);

//////////////////////////////////////////////////////////////////////////////////////swerve Motor encoders
    RelativeEncoder encoderRightA  = rightMotorA.getEncoder();
    RelativeEncoder encoderLeftA   = leftMotorA.getEncoder();
    RelativeEncoder encoderRightB = rightMotorB.getEncoder();
    RelativeEncoder encoderLeftB = leftMotorB.getEncoder();

    RelativeEncoder encoderPivotRightA  = pivotRightMotorA.getEncoder();
    RelativeEncoder encoderPivotLeftA   = pivotLeftMotorA.getEncoder();
    RelativeEncoder encoderPivotRightB = pivotRightMotorB.getEncoder();
    RelativeEncoder encoderPivotLeftB = pivotLeftMotorB.getEncoder();

    CANCoder LeftApivotAbsEncoder = new WPI_CANCoder(31);
    CANCoder RightApivotAbsEncoder = new WPI_CANCoder(32);
    CANCoder LeftBpivotAbsEncoder = new WPI_CANCoder(34);
    CANCoder RightBpivotAbsEncoder = new WPI_CANCoder(33);

//////////////////////////////////////////////////////////////////////////////////////Arm Motors

    CANSparkMax leftArmMotor = new CANSparkMax(9, kBrushless);
    CANSparkMax rightArmMotor = new CANSparkMax(8, kBrushless);


//////////////////////////////////////////////////////////////////////////////////////Collector Motors

    CANSparkMax topCollectorRoller = new CANSparkMax(2, kBrushless);
    CANSparkMax bottomCollectorRoller = new CANSparkMax(15, kBrushless);

    SparkMaxPIDController topCollectorRollerRPM = topCollectorRoller.getPIDController();
    SparkMaxPIDController bottomCollectorRollerRPM = bottomCollectorRoller.getPIDController();

/////////////////////////////////////////////////////////////////////////////////////////////////////////PID Controllers

    SparkMaxPIDController leftMotorARPM = leftMotorA.getPIDController();
    SparkMaxPIDController leftMotorBRPM = leftMotorB.getPIDController();
    SparkMaxPIDController rightMotorARPM = rightMotorA.getPIDController();
    SparkMaxPIDController rightMotorBRPM = rightMotorB.getPIDController();

    PIDController pivotLeftAPID = new PIDController(8.0e-3,0,0);
    PIDController pivotLeftBPID = new PIDController(8.0e-3,0,0);
    PIDController pivotRightAPID = new PIDController(12.0e-3,0,0);
    PIDController pivotRightBPID = new PIDController(8.0e-3,0,0);

    SwerveDriveOdometry m_odometry;

    Solenoid gripper;
    Solenoid retractor;
    Solenoid lights;

    Solenoid coneGrabLights;
    Solenoid pitchLights;
    Solenoid heartbeat;

    //AnalogInput shoulder = leftArmMotor.getAnalog(kAbsolute)
    CANCoder shoulder2 = new WPI_CANCoder(35);

    SparkMaxLimitSwitch cargoFinderForward = bottomCollectorRoller.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    SparkMaxLimitSwitch cargoFinderReverse = bottomCollectorRoller.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    SparkMaxLimitSwitch firstCargoFinderForward = topCollectorRoller.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    SparkMaxLimitSwitch firstCargoFinderReverse = topCollectorRoller.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    SparkMaxLimitSwitch armLimitSwitchForward = leftArmMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    SparkMaxLimitSwitch armLimitSwitchReverse = leftArmMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    SparkMaxLimitSwitch floorSensorLeftForward = leftMotorB.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    SparkMaxLimitSwitch floorSensorLeftReverse = leftMotorB.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    SparkMaxLimitSwitch floorSensorRightForward = rightMotorB.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    SparkMaxLimitSwitch floorSensorRightReverse = rightMotorB.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    PIDController armController = new PIDController(.01, 0.18e-2, 0);

    //TimeOfFlight timeOfFlightSensor = new TimeOfFlight(40);

    // Robot chassic dimensions, shaft to shaft.
    static final double w = 13.875;
    static final double d = 10.375;
    private static final int PH_CAN_ID = 1;

    Pose2d startingPoseOdom = defaultPose;
    PneumaticHub m_ph = new PneumaticHub(PH_CAN_ID);
    double armOffset = -4;

    public TherMOEDynamic(){

        cargoFinderForward.enableLimitSwitch(false);
        cargoFinderReverse.enableLimitSwitch(false);
        firstCargoFinderForward.enableLimitSwitch(false);
        firstCargoFinderReverse.enableLimitSwitch(false);
        armLimitSwitchForward.enableLimitSwitch(false);
        armLimitSwitchReverse.enableLimitSwitch(false);
        floorSensorRightReverse.enableLimitSwitch(false);
        floorSensorRightForward.enableLimitSwitch(false);
        floorSensorLeftForward.enableLimitSwitch(false);
        floorSensorLeftReverse.enableLimitSwitch(false);



        m_ph.enableCompressorAnalog(100,120);

        pivotLeftAPID.enableContinuousInput(-180,180);
        pivotLeftBPID.enableContinuousInput(-180,180);
        pivotRightAPID.enableContinuousInput(-180,180);
        pivotRightBPID.enableContinuousInput(-180,180);

        leftMotorA.setInverted(false);
        leftMotorB.setInverted(false);
        rightMotorA.setInverted(false);
        rightMotorB.setInverted(false);

        pivotLeftMotorA.setInverted(true);
        pivotLeftMotorB.setInverted(true);
        pivotRightMotorA.setInverted(true);
        pivotRightMotorB.setInverted(true);

        leftMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);

        topCollectorRoller.setIdleMode(CANSparkMax.IdleMode.kBrake);
        bottomCollectorRoller.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pivotLeftMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotLeftMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotRightMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotRightMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);


        leftMotorARPM.setP(7.0e-5);
        leftMotorARPM.setI(0);
        leftMotorARPM.setIZone(0);
        leftMotorARPM.setD(1.0e-4);
        leftMotorARPM.setFF(1.76182e-4);
        leftMotorARPM.setOutputRange(-1,1);

        leftMotorBRPM.setP(7.0e-5);
        leftMotorBRPM.setI(0);
        leftMotorBRPM.setD(1.0e-4);
        leftMotorBRPM.setIZone(0);
        leftMotorBRPM.setFF(1.76182e-4);
        leftMotorBRPM.setOutputRange(-1,1);

        rightMotorARPM.setP(7.0e-5);
        rightMotorARPM.setI(0);
        rightMotorARPM.setIZone(0);
        rightMotorARPM.setD(1.0e-4);
        rightMotorARPM.setFF(1.76182e-4);
        rightMotorARPM.setOutputRange(-1,1);

        rightMotorBRPM.setP(7.0e-5);
        rightMotorBRPM.setI(0);
        rightMotorBRPM.setIZone(0);
        rightMotorBRPM.setD(1.0e-4);
        rightMotorBRPM.setFF(1.76182e-4);
        rightMotorBRPM.setOutputRange(-1,1);

        topCollectorRollerRPM.setP(7.0e-5);//TODO: get electrical to tune
        topCollectorRollerRPM.setI(0);
        topCollectorRollerRPM.setIZone(0);
        topCollectorRollerRPM.setD(1.0e-4);
        topCollectorRollerRPM.setFF(1.76182e-4);
        topCollectorRollerRPM.setOutputRange(-1,1);

        bottomCollectorRollerRPM.setP(7.0e-5); //TODO: get electrical to tune
        bottomCollectorRollerRPM.setI(0);
        bottomCollectorRollerRPM.setIZone(0);
        bottomCollectorRollerRPM.setD(1.0e-4);
        bottomCollectorRollerRPM.setFF(1.76182e-4);
        bottomCollectorRollerRPM.setOutputRange(-1,1);

        leftArmMotor.setInverted(false);
        rightArmMotor.follow(leftArmMotor,true);

        topCollectorRoller.setInverted(false);
        bottomCollectorRoller.setInverted(false);

        retractor = new Solenoid(PneumaticsModuleType.REVPH,8);
        gripper   = new Solenoid(PneumaticsModuleType.REVPH,12);
        lights = new Solenoid(PneumaticsModuleType.REVPH, 9);
        coneGrabLights = new Solenoid(PneumaticsModuleType.REVPH, 14);
        pitchLights = new Solenoid(PneumaticsModuleType.REVPH, 15);
        heartbeat = new Solenoid(PneumaticsModuleType.REVPH, 13);
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////Helpful swerve commands

    @Override
    public double getMaxInchesPerSecond() {
        return 174;
    }

    @Override
    public double getMaxRadPerSec() {
        return 96/14;
    }

    @Override
    public SwerveDriveKinematics kinematics() {

        return new SwerveDriveKinematics(
                new Translation2d(w, d),//everything is in inches
                new Translation2d(w, -d),
                new Translation2d(-w, d),
                new Translation2d(-w, -d)
        );
    }
    @Override
    public void setDrive(double xspd, double yspd, double turnspd, boolean auto){this.setDrive(xspd, yspd, turnspd, auto, true);}
    @Override
    public void setDrive(double xspd, double yspd, double turnspd) {
        this.setDrive(xspd,yspd,turnspd,false, true);
    }
    @Override
    public void setDrive(double xspd, double yspd, double turnspd, boolean auto, boolean fieldCentric){
        double m_yaw = -getPigeonYaw();
        if (getRed()) m_yaw = -getPigeonYaw() + 180;
        if (auto){
            m_yaw = -getPigeonYaw();
        }
        if (!fieldCentric){m_yaw = 0;}
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xspd,
                yspd, turnspd, Rotation2d.fromDegrees(-m_yaw));
        SwerveModuleState[] moduleStates = kinematics().toSwerveModuleStates(chassisSpeeds);
        SwerveModuleState frontLeftState = moduleStates[0],
                frontRightState = moduleStates[1],
                backLeftState = moduleStates[2],
                backRightState = moduleStates[3];

        frontLeftState = optimizeSwervePivots(frontLeftState, Rotation2d.fromDegrees(getPivotLeftMotorA()));
        frontRightState = optimizeSwervePivots(frontRightState, Rotation2d.fromDegrees(getPivotRightMotorA()));
        backLeftState = optimizeSwervePivots(backLeftState, Rotation2d.fromDegrees(getPivotLeftMotorB()));
        backRightState = optimizeSwervePivots(backRightState, Rotation2d.fromDegrees(getPivotRightMotorB()));

        if (xspd == 0 && yspd == 0 && turnspd == 0) {
            stopSwerve(oldLeftA, oldRightA, oldLeftB, oldRightB);
        } else {
            swerve(frontLeftState, frontRightState, backLeftState, backRightState);
            oldLeftA = frontLeftState.angle.getDegrees();
            oldLeftB = backLeftState.angle.getDegrees();
            oldRightA = frontRightState.angle.getDegrees();
            oldRightB = backRightState.angle.getDegrees();
        }
    }

    @Override
    public Pose2d getPose() {
        double currHeading = getPigeonYaw();
        SmartDashboard.putNumber("leftAStartPos", startDists[0]);
        SmartDashboard.putNumber("rightAStartPos", startDists[1]);
        SmartDashboard.putNumber("leftBStartPos", startDists[2]);
        SmartDashboard.putNumber("rightBStartPos", startDists[3]);

        SmartDashboard.putNumber("leftAStartPivot", startPivots[0]);
        SmartDashboard.putNumber("rightAStartPivot", startPivots[1]);
        SmartDashboard.putNumber("leftBStartPivot", startPivots[2]);
        SmartDashboard.putNumber("rightBStartPivot", startPivots[3]);

        SmartDashboard.putNumber("leftACurrPos", getDriveDistanceInchesLeftA());
        SmartDashboard.putNumber("rightACurrPos",getDriveDistanceInchesRightA());
        SmartDashboard.putNumber("leftBCurrPos", getDriveDistanceInchesLeftB());
        SmartDashboard.putNumber("rightBCurrPos", getDriveDistanceInchesRightB());

        SmartDashboard.putNumber("leftACurrPivot", getPivotLeftMotorA());
        SmartDashboard.putNumber("rightACurrPivot", getPivotRightMotorA());
        SmartDashboard.putNumber("leftBCurrPivot", getPivotLeftMotorB());
        SmartDashboard.putNumber("rightBCurrPivot", getPivotRightMotorB());
        SmartDashboard.putNumber("startHeading", startHeading);
        Pose2d myPose = m_odometry.update(Rotation2d.fromDegrees(currHeading),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(getDriveDistanceInchesLeftA(), Rotation2d.fromDegrees(getPivotLeftMotorA())),
                        new SwerveModulePosition(getDriveDistanceInchesRightA(), Rotation2d.fromDegrees(getPivotRightMotorA())),
                        new SwerveModulePosition(getDriveDistanceInchesLeftB(), Rotation2d.fromDegrees(getPivotLeftMotorB())),
                        new SwerveModulePosition(getDriveDistanceInchesRightB(), Rotation2d.fromDegrees(getPivotRightMotorB()))
                });
        //Pose2d correctPose = new Pose2d(myPose.getX(),(2*startingPoseOdom.getY() - myPose.getY()), myPose.getRotation());
        SmartDashboard.putNumber("xPose", myPose.getX());
        SmartDashboard.putNumber("yPose", myPose.getY());
        SmartDashboard.putNumber("rotation", myPose.getRotation().getDegrees());
        return myPose;
    }



    public void setPose(Pose2d startPose) {
        startingPoseOdom = startPose;
        m_odometry = new SwerveDriveOdometry(
                kinematics(), startPose.getRotation(),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(startDists[0], Rotation2d.fromDegrees(startPivots[0])),
                        new SwerveModulePosition(startDists[1], Rotation2d.fromDegrees(startPivots[1])),
                        new SwerveModulePosition(startDists[2], Rotation2d.fromDegrees(startPivots[2])),
                        new SwerveModulePosition(startDists[3], Rotation2d.fromDegrees(startPivots[3]))
                }, startPose);
    }
    @Override
    public void resetStartHeading(){
        startHeading = pigeon.getYaw();
    }

    @Override
    public void setPose() {
        this.setPose(defaultPose);
    }

//////////////////////////////////////////////////////////////////////////////////////////////////yaw pitch and roll- navX and pigeon
    @Override
    public double getYaw() {
        return navx.getYaw();
    }

    @Override
    public double getRoll() {
        return navx.getRoll();
    }

    @Override
    public double getPitch() {
        return navx.getPitch();
    }

    @Override
    public void resetAttitude() {
        navx.reset();
    }

////////////////////////////////////////////////////////////////////////////////////Pigeon Commands

    @Override
    public double getPigeonYaw() {
        return pigeon.getYaw();
    }

    @Override
    public double getPigeonRoll() {
        return pigeon.getRoll();
    }

    @Override
    public double getPigeonPitch() {
        return pigeon.getPitch();
    }

    @Override
    public double getAbsoluteCompassHeadingPigeon() {
        return pigeon.getAbsoluteCompassHeading();
    }

    @Override
    public void resetPigeon() {
        pigeon.reset();
    }

    @Override
    public void setPigeonYaw(double startYaw){
        pigeon.setYaw(startYaw);
    }
//////////////////////////////////////////////////////////////////////////////////////////drive motor encoders
    @Override
    public double encoderLeftADriveTicksPerInch() {
        return 6.75/12.375*1.03/1.022;
    }

    @Override
    public double encoderLeftBDriveTicksPerInch() {
        return encoderLeftADriveTicksPerInch();
    }

    @Override
    public double encoderRightADriveTicksPerInch() {
        return encoderLeftADriveTicksPerInch();
    }

    @Override
    public double encoderRightBDriveTicksPerInch() {
        return encoderLeftADriveTicksPerInch();
    }

    @Override
    public double convertInchpsToRPM() {
        return 32.73*1.03/1.022;
    }

    @Override
    public double encoderTicksLeftDriveA() {
        SmartDashboard.putNumber("encoderTicksLeftA", encoderLeftA.getPosition());
        return encoderLeftA.getPosition();
    }

    @Override
    public double encoderTicksLeftDriveB() {
        SmartDashboard.putNumber("encoderTicksLeftB", encoderLeftB.getPosition());
        return encoderLeftB.getPosition();
    }

    @Override
    public double encoderTicksRightDriveA() {
        SmartDashboard.putNumber("encoderTicksRightA", encoderRightA.getPosition());
        return encoderRightA.getPosition();
    }

    @Override
    public double encoderTicksRightDriveB() {
        SmartDashboard.putNumber("encoderTicksRightB", encoderRightB.getPosition());
        return encoderRightB.getPosition();
    }
////////////////////////////////////////////////////////////////////////////////////////////drive motor outputs
    @Override
    public void setLeftDriveAPowerPercentage(double power) {
        leftMotorA.set(power);
    }
    @Override
    public void setLeftDriveBPowerPercentage(double power) {
        leftMotorB.set(power);
    }

    @Override
    public void setRightDriveAPowerPercentage(double power) {
        rightMotorA.set(power);
    }

    @Override
    public void setRightDriveBPowerPercentage(double power) {
        rightMotorB.set(power);
    }

    @Override
    public void setLeftDriveARPM(double rpm) {
        SmartDashboard.putNumber("leftMotorArpm", encoderLeftA.getVelocity());
        SmartDashboard.putNumber("leftAWantRPM", rpm);
        leftMotorARPM.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setLeftDriveBRPM(double rpm) {
        SmartDashboard.putNumber("leftMotorBrpm", encoderLeftB.getVelocity());
        SmartDashboard.putNumber("leftBWantRPM", rpm);
        leftMotorBRPM.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setRightDriveARPM(double rpm) {
        SmartDashboard.putNumber("rightMotorArpm", encoderRightA.getVelocity());
        SmartDashboard.putNumber("rightAWantRPM", rpm);
        rightMotorARPM.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setRightDriveBRPM(double rpm) {
        SmartDashboard.putNumber("rightMotorBrpm", encoderRightB.getVelocity());
        SmartDashboard.putNumber("rightBWantRPM", rpm);
        rightMotorBRPM.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public double getLeftACurrent() {
        return leftMotorA.getOutputCurrent();
    }

    @Override
    public double getRightACurrent() {
        return rightMotorA.getOutputCurrent();
    }

    @Override
    public double getLeftBCurrent() {
        return leftMotorB.getOutputCurrent();
    }

    @Override
    public double getRightBCurrent() {
        return rightMotorB.getOutputCurrent();
    }


    ////////////////////////////////////////////////////////////////////////////////////////////Pivot motor encoder commands
    @Override
    public void resetPIDPivot() {
        pivotLeftAPID.reset();
        pivotLeftBPID.reset();
        pivotRightAPID.reset();
        pivotRightBPID.reset();
    }
    @Override
    public double getPivotLeftMotorA() {
        if (LeftApivotAbsEncoder.getAbsolutePosition()+45 < 0){
            return (LeftApivotAbsEncoder.getAbsolutePosition() + 45 - 180)%360 + 180;
        }
        else{
            return (LeftApivotAbsEncoder.getAbsolutePosition() + 45 + 180)%360 - 180;
        }
    }

    @Override
    public double getPivotLeftMotorB() {
        if (LeftBpivotAbsEncoder.getAbsolutePosition() + 135 < 0){
            return (LeftBpivotAbsEncoder.getAbsolutePosition() + 135 -180)%360 + 180;
        }
        else{
            return (LeftBpivotAbsEncoder.getAbsolutePosition() + 135 +180)%360 - 180;
        }
    }

    @Override
    public double getPivotRightMotorA() {
        if (RightApivotAbsEncoder.getAbsolutePosition() - 45 < 0){
            return (RightApivotAbsEncoder.getAbsolutePosition() - 45 - 180)%360 + 180;
        }
        else{
            return (RightApivotAbsEncoder.getAbsolutePosition() - 45 + 180)%360 - 180;
        }
    }

    @Override
    public double getPivotRightMotorB() {
        if (RightBpivotAbsEncoder.getAbsolutePosition() - 135 < 0){
            return (RightBpivotAbsEncoder.getAbsolutePosition() - 135 - 180)%360+180;
        }
        else{
            return (RightBpivotAbsEncoder.getAbsolutePosition() - 135 + 180)%360-180;
        }
    }
////////////////////////////////////////////////////////////////////////////////////////////////pivot motor outputs
    @Override
    public void setPivotLeftMotorA(double Pivot) {
        SmartDashboard.putNumber("PivotLeftMotorADesiredPivot", Pivot);
        SmartDashboard.putBoolean("LeftAAligned", Math.abs(Pivot-getPivotLeftMotorA()) <= 4);
        //PivotMotorPIDLeftA.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
        pivotLeftMotorA.set(pivotLeftAPID.calculate(-Pivot + getPivotLeftMotorA()));
    }

    @Override
    public void setPivotLeftMotorB(double Pivot) {
        SmartDashboard.putNumber("PivotLeftMotorBDesiredPivot", Pivot);
        SmartDashboard.putBoolean("LeftBAligned", Math.abs(Pivot-getPivotLeftMotorB()) <= 4);
        //PivotMotorPIDLeftB.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
        pivotLeftMotorB.set(pivotLeftBPID.calculate(-Pivot + getPivotLeftMotorB()));
    }

    @Override
    public void setPivotRightMotorA(double Pivot) {
        SmartDashboard.putNumber("PivotRightMotorADesiredPivot", Pivot);
        SmartDashboard.putBoolean("RightAAligned", Math.abs(Pivot-getPivotRightMotorA()) <= 4);
        //PivotMotorPIDRightA.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
        pivotRightMotorA.set(pivotRightAPID.calculate(-Pivot + getPivotRightMotorA()));
    }

    @Override
    public void setPivotRightMotorB(double Pivot) {
        SmartDashboard.putNumber("PivotRightMotorBDesiredPivot", Pivot);
        SmartDashboard.putBoolean("RightBAligned", Math.abs(Pivot-getPivotRightMotorB()) <= 4);
        //PivotMotorPIDRightB.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
        pivotRightMotorB.set(pivotRightBPID.calculate(-Pivot + getPivotRightMotorB()));
    }

//////////////////////////////////////////////////////////////////////////////////////collector motor outputs

    @Override
    public void setBottomRollerPower(double power) {
        bottomCollectorRoller.set(power);
    }

    @Override
    public void setTopRollerPower(double power) {
        topCollectorRoller.set(power);
    }

    @Override
    public void collect(double rpm) {
        this.collect(rpm, false);
    }

    @Override
    public void collect(double rpm, boolean autoMode) {
        if (cargoInCollector() && rpm > 0 && !autoMode){
            rpm = 0;
        }
        setTopRollerRPM(rpm);
        setBottomRollerRPM(rpm);
    }

    @Override
    public void setBottomRollerRPM(double rpm) {
        bottomCollectorRollerRPM.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setTopRollerRPM(double rpm) {
        topCollectorRollerRPM.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void raiseTopRoller(boolean up) {
        boolean drop = false;
        if (!up) drop = true;
        retractor.set(drop);
    }

    @Override
    public double getTopRollerPosition() {
        return super.getTopRollerPosition();
    }

    @Override
    public boolean cargoInCollector() {
        return cargoFinderForward.isPressed();
    }

    @Override
    public boolean cargoDetected(){return firstCargoFinderForward.isPressed();}
    @Override
    public boolean armHitLimit(){return armLimitSwitchReverse.isPressed();}
////////////////////////////////////////////////////////////////////////////////////arm motor commands
    @Override
    public void rightArmPower(double power) {
        rightArmMotor.set(power);
    }

    @Override
    public void leftArmPower(double power) {
        leftArmMotor.set(power);
    }

    @Override
    public void moveArm(double power) {

        if (getPotDegrees() <= lowestAngle && power < 0){
            power = 0;
        }
        if (getPotDegrees() >= greatestAngle && power > 0){
            power = 0;
        }
        leftArmMotor.set(power);
        rightArmMotor.set(power);
    }
    @Override
    public void liftArm(){
        boolean openGrip = false;
        double armPower = 0;
        if (Math.abs(getArmPosition()- MAX_ARM_HEIGHT) <= 3){
            armPower = 0;
        }
        else{
            armPower = .02*(-getArmPosition() + MAX_ARM_HEIGHT);
            if (armPower < 0){
                armPower = Math.max(-.5, armPower);
            }
            else{
                armPower = Math.min(.5, armPower);
            }
        }
        moveArm(armPower);
        openGripper(openGrip);
    }

    @Override
    public void dropArm(){
        boolean openGrip = true;
        double armPower = 0;
        if (Math.abs(getArmPosition()- MIN_ARM_HEIGHT) <= 3){
            armPower = 0;
        }
        else{
            armPower = .02*(-getArmPosition() + MIN_ARM_HEIGHT);
            if (armPower < 0){
                armPower = Math.max(-.5, armPower);
            }
            else{
                armPower = Math.min(.5, armPower);
            }
        }
        moveArm(armPower);
        openGripper(openGrip);
    }

    @Override
    public double getArmPosition() {
        //SmartDashboard.putNumber("potentiometer Reading - shoulder1", shoulder.getPosition());
        SmartDashboard.putNumber("pot Reading - shoulder 2", shoulder2.getAbsolutePosition());
        double theta =  getPotRadians();
        double height = MAX_ARM_HEIGHT - armRadius + armRadius*Math.sin(theta);
        if (theta > Math.PI/2) height = MAX_ARM_HEIGHT + armRadius*Math.sin(theta-Math.PI/2);
        return height;
    }

    @Override
    public double getPotRadians(){
        return getPotDegrees()*Math.PI/180;
    }
    @Override
    public double getPotDegrees(){
        return -shoulder2.getAbsolutePosition() - armOffset;
    }

    @Override
    public void stackCargo(double zPos) {
        double armPower = 0;
        if (Math.abs(getArmPosition()- zPos) <= 3){ //6" (+/- 3)tolerance
            armPower = 0;
        }
        else{
            armPower = .02*(-getArmPosition() + zPos);
            if (armPower < 0){
                armPower = Math.max(-.5, armPower);
            }
            else{
                armPower = Math.min(.5, armPower);
            }
        }
        moveArm(armPower);
    }

    @Override
    public void holdArmPosition(double pos){
        pos = Math.max(lowestAngle, pos);
        double armPower = armController.calculate(getPotDegrees() - pos);
        if (armPower < 0){
            armPower = Math.max(-.2, armPower);
        }
        else{
            armPower = Math.min(.4, armPower);
        }
        moveArm(armPower);
    }

    @Override
    public void resetArmPID() {
        armController.reset();
    }

    /////////////////////////////////////////////////////////////////////////////////////gripper commands
    @Override
    public void openGripper(boolean open) {
        gripper.set(open);
    }


    @Override
    public boolean gripperIsOpen() {
        return gripper.get();
    }
    ///////////////////////////////////////////////////////////////////////////////////lights

    @Override
    public void setLightsOn(boolean on) {
        lights.set(on);
    }

    @Override
    public void coneGrabInAction(boolean on) {
        coneGrabLights.set(on);
    }

    @Override
    public void robotTipping(boolean tip) {
        pitchLights.set(tip);
    }

    @Override
    public void robotHeartbeat(boolean beat) {
        heartbeat.set(beat);
    }

    //////////////////////////////////////////////////////////////////////////////////floor sensors

    @Override
    public boolean getLeftFloorSensor() {
        return floorSensorLeftForward.isPressed();
    }

    @Override
    public boolean getRightFloorSensor() {
        return floorSensorRightForward.isPressed();
    }
    /////////////////////////////////////////////////////////////////////////////////ultrasonics add after swerve tested

}