package frc.robot.generic;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class    swerveBot extends GenericRobot{
    private final Timer m_timer = new Timer();
    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);
    WPI_Pigeon2 pigeon = new WPI_Pigeon2(0);



    ///////////////////////////////////////////////////////////////////////////////////////////////////////Motor definitions
    CANCoder LeftApivotAbsEncoder = new WPI_CANCoder(1);
    CANCoder RightBpivotAbsEncoder = new WPI_CANCoder(3);
    CANCoder RightApivotAbsEncoder = new WPI_CANCoder(2);
    CANCoder LeftBpivotAbsEncoder = new WPI_CANCoder(4);

    CANSparkMax leftMotorA        = new CANSparkMax(11, kBrushless);
    CANSparkMax pivotLeftMotorA   = new CANSparkMax(10, kBrushless);

    CANSparkMax leftMotorB        = new CANSparkMax( 19, kBrushless);
    CANSparkMax pivotLeftMotorB   = new CANSparkMax(18, kBrushless);

    CANSparkMax rightMotorA       = new CANSparkMax(9, kBrushless);
    CANSparkMax pivotRightMotorA   = new CANSparkMax(8, kBrushless);

    CANSparkMax rightMotorB       = new CANSparkMax(1, kBrushless);
    CANSparkMax pivotRightMotorB   = new CANSparkMax(20, kBrushless);

    RelativeEncoder encoderRightA  = rightMotorA.getEncoder();
    RelativeEncoder encoderLeftA   = leftMotorA.getEncoder();
    RelativeEncoder encoderRightB = rightMotorB.getEncoder();
    RelativeEncoder encoderLeftB = leftMotorB.getEncoder();

    RelativeEncoder encoderPivotRightA  = pivotRightMotorA.getEncoder();
    RelativeEncoder encoderPivotLeftA   = pivotLeftMotorA.getEncoder();
    RelativeEncoder encoderPivotRightB = pivotRightMotorB.getEncoder();
    RelativeEncoder encoderPivotLeftB = pivotLeftMotorB.getEncoder();

/////////////////////////////////////////////////////////////////////////////////////////////////////////PID Controllers
    SparkMaxPIDController PivotMotorPIDLeftA = pivotLeftMotorA.getPIDController();
    SparkMaxPIDController PivotMotorPIDRightA = pivotRightMotorA.getPIDController();
    SparkMaxPIDController PivotMotorPIDLeftB = pivotLeftMotorB.getPIDController();
    SparkMaxPIDController PivotMotorPIDRightB = pivotRightMotorB.getPIDController();

    SparkMaxPIDController leftMotorARPM = leftMotorA.getPIDController();
    SparkMaxPIDController leftMotorBRPM = leftMotorB.getPIDController();
    SparkMaxPIDController rightMotorARPM = rightMotorA.getPIDController();
    SparkMaxPIDController rightMotorBRPM = rightMotorB.getPIDController();

    PIDController pivotLeftAPID = new PIDController(8.0e-3,0,0);
    PIDController pivotLeftBPID = new PIDController(8.0e-3,0,0);
    PIDController pivotRightAPID = new PIDController(8.0e-3,0,0);
    PIDController pivotRightBPID = new PIDController(8.0e-3,0,0);

    TimeOfFlight timeOfFlightSensor = new TimeOfFlight(0);
    SwerveDriveOdometry m_odometry;

    /////////////////////////////////////////////////////////////Light Sensors
    DigitalInput leftLightSensor = new DigitalInput(0);
    DigitalInput rightLightSensor = new DigitalInput(1);

    Pose2d startingPoseOdom = defaultPose;



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////Further Motor Stuff
    public swerveBot(){

        pivotLeftAPID.enableContinuousInput(-180,180);
        pivotLeftBPID.enableContinuousInput(-180,180);
        pivotRightAPID.enableContinuousInput(-180,180);
        pivotRightBPID.enableContinuousInput(-180,180);

        PivotMotorPIDLeftA.setP(0.1);
        PivotMotorPIDLeftB.setP(0.1);
        PivotMotorPIDRightA.setP(0.1);
        PivotMotorPIDRightB.setP(0.1);

        PivotMotorPIDLeftA.setI(0);
        PivotMotorPIDLeftB.setI(0);
        PivotMotorPIDRightA.setI(0);
        PivotMotorPIDRightB.setI(0);

        PivotMotorPIDLeftA.setD(0);
        PivotMotorPIDLeftB.setD(0);
        PivotMotorPIDRightA.setD(0);
        PivotMotorPIDRightB.setD(0);

        PivotMotorPIDLeftA.setFF(0);
        PivotMotorPIDLeftB.setFF(0);
        PivotMotorPIDRightA.setFF(0);
        PivotMotorPIDRightB.setFF(0);

        PivotMotorPIDLeftA.setOutputRange(-1, 1);
        PivotMotorPIDLeftB.setOutputRange(-1, 1);
        PivotMotorPIDRightA.setOutputRange(-1, 1);
        PivotMotorPIDRightB.setOutputRange(-1, 1);

        leftMotorA.setInverted(false);
        leftMotorB.setInverted(false);
        rightMotorA.setInverted(false);
        rightMotorB.setInverted(false);

        pivotLeftMotorA.setInverted(true);
        pivotLeftMotorB.setInverted(true);
        pivotRightMotorA.setInverted(true);
        pivotRightMotorB.setInverted(true);

        pivotLeftMotorA.follow(CANSparkMax.ExternalFollower.kFollowerDisabled, 0);
        pivotLeftMotorB.follow(CANSparkMax.ExternalFollower.kFollowerDisabled, 0);
        pivotRightMotorA.follow(CANSparkMax.ExternalFollower.kFollowerDisabled, 0);
        pivotRightMotorB.follow(CANSparkMax.ExternalFollower.kFollowerDisabled, 0);

        leftMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pivotLeftMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotLeftMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotRightMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotRightMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftMotorARPM.setP(1.5e-4);
        leftMotorARPM.setI(0);
        leftMotorARPM.setIZone(0);
        leftMotorARPM.setD(1.0e-4);
        leftMotorARPM.setFF(1.76182e-4);
        leftMotorARPM.setOutputRange(-1,1);

        leftMotorBRPM.setP(1.5e-4);
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

    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////Implementation
    @Override
    public double getMaxInchesPerSecond() {
        return 120; //TODO: verify this
    }
    @Override
    public double getMaxRadPerSec(){
        return 120/14; //TODO: idk if this even matters
    }
    @Override
    public SwerveDriveKinematics kinematics() {
        return new SwerveDriveKinematics(
                new Translation2d(14, 14),//everything is in inches
                new Translation2d(14, -14),
                new Translation2d(-14, 14),
                new Translation2d(-14, -14)
        );
    }

    @Override
    public void SwerveAutoReset() {
        m_timer.reset();
        m_timer.start();
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

    @Override
    public void setPose(Pose2d startPose){
        startingPoseOdom = startPose;
        m_odometry = new SwerveDriveOdometry(
                kinematics(), Rotation2d.fromDegrees(startHeading),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(startDists[0], Rotation2d.fromDegrees(startPivots[0])),
                        new SwerveModulePosition(startDists[1], Rotation2d.fromDegrees(startPivots[1])),
                        new SwerveModulePosition(startDists[2], Rotation2d.fromDegrees(startPivots[2])),
                        new SwerveModulePosition(startDists[3], Rotation2d.fromDegrees(startPivots[3]))
                }, startPose);
    }
    @Override
    public void setPose(){
        this.setPose(defaultPose);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////navx commands
    @Override
    public double getYaw() {
        return navx.getYaw();
    }

    @Override
    public double getRoll() {
        return navx.getPitch();
    }

    @Override
    public double getPitch() {
        return -navx.getRoll();
    }

    @Override
    public void resetAttitude() {
        navx.reset();
    }

////////////////////////////////////////////////////////////////////////////////////////////////////pigeon commmands

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
    public void setPigeonYaw(double startYaw) {
        pigeon.setYaw(startYaw);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////drive motors



//////////////////////////////////////////////////////////////////////////drive encoders
    @Override
    public double encoderLeftADriveTicksPerInch() {
        return 6.75/12.375*1.03;
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
    public double convertInchpsToRPM() {
        return 32.73*1.03;
    }

    @Override
    public double encoderRightBDriveTicksPerInch() {
        return encoderLeftADriveTicksPerInch();
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
        SmartDashboard.putNumber("conversionFactor", encoderRightA.getPositionConversionFactor());
        return encoderRightA.getPosition();
    }

    @Override
    public double encoderTicksRightDriveB() {
        SmartDashboard.putNumber("encoderTicksRightB", encoderRightB.getPosition());
        return encoderRightB.getPosition();
    }
//////////////////////////////////////////////////////////////////////////////////////drive power
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

///////////////////////////////////////////////////////////////////////////drive motor
    @Override
    public void setDrive(double xspd, double yspd, double turnspd, boolean auto){this.setDrive(xspd,yspd,turnspd,auto,true);}
    @Override
    public void setDrive(double xspd, double yspd, double turnspd) {
        this.setDrive(xspd,yspd,turnspd,false, false);
    }
    @Override
    public void setDrive(double xspd, double yspd, double turnspd, boolean auto, boolean fieldCentric){
        double m_yaw = getYaw();
        if (auto){
            m_yaw = -getPigeonYaw();
        }
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////Pivot commands
    @Override
    public void resetPIDPivot() {
        setOffsetLeftA();
        setOffsetRightA();
        setOffsetLeftB();
        setOffsetRightB();
        pivotLeftAPID.reset();
        pivotLeftBPID.reset();
        pivotRightAPID.reset();
        pivotRightBPID.reset();
    }
//////////////////////////////////////////////////////////////////////////////pivot encoders

    @Override
    public double rawEncoderLeftA() {
        return encoderPivotLeftA.getPosition()/rotOverDeg;
    }

    @Override
    public double rawEncoderLeftB() {
        return encoderPivotLeftB.getPosition()/rotOverDeg;
    }

    @Override
    public double rawEncoderRightA() {
        return encoderPivotRightA.getPosition()/rotOverDeg;
    }

    @Override
    public double rawEncoderRightB() {
        return encoderPivotRightB.getPosition()/rotOverDeg;
    }

    @Override
    public void setOffsetLeftA() {
        offsetLeftA = encoderPivotLeftA.getPosition()/rotOverDeg - getPivotLeftMotorA();
        SmartDashboard.putNumber("offsetLeftA", offsetLeftA);
    }

    @Override
    public void setOffsetLeftB() {
        offsetLeftB = encoderPivotLeftB.getPosition()/rotOverDeg - getPivotLeftMotorB();
        SmartDashboard.putNumber("offsetLeftB", offsetLeftB);
    }

    @Override
    public void setOffsetRightA() {
        offsetRightA = encoderPivotRightA.getPosition()/rotOverDeg - getPivotRightMotorA();
        SmartDashboard.putNumber("offsetRightA", offsetRightA);
    }

    @Override
    public void setOffsetRightB() {
        offsetRightB = encoderPivotRightB.getPosition()/rotOverDeg - getPivotRightMotorB();
        SmartDashboard.putNumber("offsetRightB", offsetRightB);
    }

    @Override
    public void swerve(SwerveModuleState frontLeft, SwerveModuleState frontRight, SwerveModuleState backLeft, SwerveModuleState backRight) {
        super.swerve(frontLeft, frontRight, backLeft, backRight);
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
///////////////////////////////////////////////////////////////////////////pivot motor power
    double rotOverDeg = (150.0/7)/360;
    @Override
    public void setPivotLeftMotorA(double Pivot) {

        //Pivot += offsetLeftA;
        SmartDashboard.putNumber("PivotLeftMotorADesiredPivot", Pivot);
        SmartDashboard.putBoolean("LeftAAligned", Math.abs(Pivot-getPivotLeftMotorA()) <= 4);
        //PivotMotorPIDLeftA.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
        pivotLeftMotorA.set(pivotLeftAPID.calculate(-Pivot + getPivotLeftMotorA()));

    }

    @Override
    public void setPivotLeftMotorB(double Pivot) {
        //Pivot += offsetLeftB;
        SmartDashboard.putNumber("PivotLeftMotorBDesiredPivot", Pivot);
        SmartDashboard.putBoolean("LeftBAligned", Math.abs(Pivot-getPivotLeftMotorB()) <= 4);
        //PivotMotorPIDLeftB.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
        pivotLeftMotorB.set(pivotLeftBPID.calculate(-Pivot + getPivotLeftMotorB()));
    }

    @Override
    public void setPivotRightMotorA(double Pivot) {
       // Pivot += offsetRightA;
        SmartDashboard.putNumber("PivotRightMotorADesiredPivot", Pivot);
        SmartDashboard.putBoolean("RightAAligned", Math.abs(Pivot-getPivotRightMotorA()) <= 4);
        //PivotMotorPIDRightA.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
        pivotRightMotorA.set(pivotRightAPID.calculate(-Pivot + getPivotRightMotorA()));
    }

    @Override
    public void setPivotRightMotorB(double Pivot) {
        //Pivot += offsetRightB;
        SmartDashboard.putNumber("PivotRightMotorBDesiredPivot", Pivot);
        SmartDashboard.putBoolean("RightBAligned", Math.abs(Pivot-getPivotRightMotorB()) <= 4);
        //PivotMotorPIDRightB.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
        pivotRightMotorB.set(pivotRightBPID.calculate(-Pivot + getPivotRightMotorB()));
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////Collector Code


    @Override
    public void setBottomRollerPower(double power) {
        super.setBottomRollerPower(power);
    }

    @Override
    public void setTopRollerPower(double power) {
        super.setTopRollerPower(power);
    }

    @Override
    public void collect(double rpm) {
        setTopRollerRPM(rpm*.8);
        setBottomRollerRPM(rpm);
    }

    @Override
    public void setBottomRollerRPM(double rpm) {
        super.setBottomRollerRPM(rpm);
    }

    @Override
    public void setTopRollerRPM(double rpm) {
        super.setTopRollerRPM(rpm);
    }


    @Override
    public double getTopRollerPosition() {
        return super.getTopRollerPosition();
    }

    @Override
    public boolean cargoInCollector() {
        return super.cargoInCollector();
    }

    //////////////////////////////////////////////////////////////////////////////////////Arm Code

    @Override
    public void rightArmPower(double power) {
        super.rightArmPower(power);
    }

    @Override
    public void leftArmPower(double power) {
        super.leftArmPower(power);
    }

    @Override
    public void moveArm(double power) {
        super.moveArm(power);
    }

    @Override
    public double getArmPosition() {
        return super.getArmPosition();
    }

    //////////////////////////////////////////////////////////////////////////////////////////Gripper Code

    @Override
    public boolean gripperIsOpen() {
        return super.gripperIsOpen();
    }

    /////////////////////////////////////////////////////////////////////////////Light Sensor Code

    /*@Override
    public boolean getLeftLightSensor(){return leftLightSensor.get();}

    @Override
    public boolean getRightLightSensor(){return rightLightSensor.get();}

    ///////////////////////////////////////////////////////////////////////////////////TimeOfFlight Code

    @Override
    public double getTOFDistance(){
        return timeOfFlightSensor.getRange();
    }

    @Override
    public double getTOFAmbientLightLevel(){
        return timeOfFlightSensor.getAmbientLightLevel();
    }
    public boolean getRightLightSensor(){return rightLightSensor.get();}*/
}
