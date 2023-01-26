package frc.robot.generic;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class swerveBot implements GenericRobot{
    private final Timer m_timer = new Timer();
    double offsetLeftA, offsetLeftB, offsetRightA, offsetRightB;
    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

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

        leftMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pivotLeftMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotLeftMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotRightMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotRightMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);


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

    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////Implementation
    @Override
    public double getMaxInchesPerSecond() {
        return 79.5; //TODO: verify this
    }
    @Override
    public double getMaxRadPerSec(){
        return 5.68; //TODO: idk if this even matters
    }
    @Override
    public SwerveDriveKinematics kinematics() {
        return new SwerveDriveKinematics(
                new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(14)),
                new Translation2d(Units.inchesToMeters(14), -Units.inchesToMeters(14)),
                new Translation2d(-Units.inchesToMeters(14), Units.inchesToMeters(14)),
                new Translation2d(-Units.inchesToMeters(14), -Units.inchesToMeters(14))
        );
    }

    @Override
    public void SwerveAutoReset() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void SwerveControllerCommand(Trajectory trajectory, Pose2d pose, SwerveDriveKinematics kinematics, PIDController xController,
                                        PIDController yController, PIDController thetaController) {

       var desiredState = trajectory.sample(m_timer.get());
       var desiredVel = desiredState.velocityMetersPerSecond;
       var desiredPose = desiredState.poseMeters;
       SmartDashboard.putNumber("poseX", desiredPose.getX());
       SmartDashboard.putNumber("poseY", desiredPose.getY());
       SmartDashboard.putNumber("posrotation", desiredPose.getRotation().getDegrees());
       var xVelocity = xController.calculate(pose.getX(), desiredPose.getX());
       var yVelocity = yController.calculate(pose.getY(), desiredPose.getY());
       var angularVel = thetaController.calculate(pose.getRotation().getDegrees(), desiredPose.getRotation().getDegrees());

       ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, angularVel);
       SwerveModuleState[] moduleStates = kinematics().toSwerveModuleStates(targetChassisSpeeds);
        SwerveModuleState frontLeftState = moduleStates[0],
                frontRightState = moduleStates[1],
                backLeftState = moduleStates[2],
                backRightState = moduleStates[3];

        frontLeftState = SwerveModuleState.optimize(frontLeftState, Rotation2d.fromDegrees(getPivotLeftMotorA()));
        frontRightState = SwerveModuleState.optimize(frontRightState, Rotation2d.fromDegrees(getPivotRightMotorA()));
        backLeftState = SwerveModuleState.optimize(backLeftState, Rotation2d.fromDegrees(getPivotLeftMotorB()));
        backRightState = SwerveModuleState.optimize(backRightState, Rotation2d.fromDegrees(getPivotRightMotorB()));

        swerve(frontLeftState, frontRightState, backLeftState, backRightState);
                 

    }

    @Override
    public Pose2d getPose(double startHeading, double currHeading, double[] startDistances, double[] startPivots, Pose2d startPose) {
        SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
                kinematics(), Rotation2d.fromDegrees(-startHeading),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(startDistances[0], Rotation2d.fromDegrees(-startPivots[0])),
                        new SwerveModulePosition(startDistances[1], Rotation2d.fromDegrees(-startPivots[1])),
                        new SwerveModulePosition(startDistances[2], Rotation2d.fromDegrees(-startPivots[2])),
                        new SwerveModulePosition(startDistances[3], Rotation2d.fromDegrees(-startPivots[3]))
                }, startPose);
        Pose2d myPose = m_odometry.update(Rotation2d.fromDegrees(-currHeading),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(getDriveDistanceInchesLeftA(), Rotation2d.fromDegrees(-getPivotLeftMotorA())),
                        new SwerveModulePosition(getDriveDistanceInchesRightA(), Rotation2d.fromDegrees(-getPivotRightMotorA())),
                        new SwerveModulePosition(getDriveDistanceInchesLeftB(), Rotation2d.fromDegrees(-getPivotLeftMotorB())),
                        new SwerveModulePosition(getDriveDistanceInchesRightB(), Rotation2d.fromDegrees(-getPivotRightMotorB()))
        });
        SmartDashboard.putNumber("xPose", myPose.getX());
        SmartDashboard.putNumber("yPose", myPose.getY());
        SmartDashboard.putNumber("rotation", myPose.getRotation().getDegrees());
        return myPose;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////navx commands
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
////////////////////////////////////////////////////////////////////////////////////////////////////drive motors

//////////////////////////////////////////////////////////////////////////drive encoders
    @Override
    public double encoderLeftADriveTicksPerInch() {
        return 62.86;
    }

    @Override
    public double encoderLeftBDriveTicksPerInch() {
        return 62.86;
    }

    @Override
    public double encoderRightADriveTicksPerInch() {
        return 62.86;
    }

    @Override
    public double encoderRightBDriveTicksPerInch() {
        return 62.86;
    }

    @Override
    public double encoderTicksLeftDriveA() {
        return encoderLeftA.getPosition();
    }

    @Override
    public double encoderTicksLeftDriveB() {
        return encoderLeftB.getPosition();
    }

    @Override
    public double encoderTicksRightDriveA() {
        return encoderRightA.getPosition();
    }

    @Override
    public double encoderTicksRightDriveB() {
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
        GenericRobot.super.swerve(frontLeft, frontRight, backLeft, backRight);
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


}
