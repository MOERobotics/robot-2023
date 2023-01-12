package frc.robot.generic;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class swerveBot implements GenericRobot{

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

////////////////////////////////////////////////////////////////////////////////Motor definitions
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////PID Controllers
    PIDController PivotMotorPIDLeftA = new PIDController(16.0e-3,1.0e-7,3.0e-5);
    PIDController PivotMotorPIDRightA = new PIDController(16.0e-3,1.0e-7,3.0e-5);
    PIDController PivotMotorPIDLeftB = new PIDController(16.0e-3,1.0e-7,3.0e-5);
    PIDController PivotMotorPIDRightB = new PIDController(16.0e-3,1.0e-7,3.0e-5);

    SparkMaxPIDController leftMotorARPM = leftMotorA.getPIDController();
    SparkMaxPIDController leftMotorBRPM = leftMotorB.getPIDController();
    SparkMaxPIDController rightMotorARPM = rightMotorA.getPIDController();
    SparkMaxPIDController rightMotorBRPM = rightMotorB.getPIDController();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////Further Motor Stuff
    public swerveBot(){

        PivotMotorPIDLeftA.enableContinuousInput(-180,180);
        PivotMotorPIDLeftB.enableContinuousInput(-180,180);
        PivotMotorPIDRightA.enableContinuousInput(-180,180);
        PivotMotorPIDRightB.enableContinuousInput(-180,180);

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


        leftMotorARPM.setP(3.0e-4);
        leftMotorARPM.setI(1.0e-7);
        leftMotorARPM.setD(9.0e-4);
        leftMotorARPM.setFF(1.7e-4);
        leftMotorARPM.setIZone(500);
        leftMotorARPM.setDFilter(0);
        leftMotorARPM.setOutputRange(0,1);

        leftMotorBRPM.setP(1.8e-4);
        leftMotorBRPM.setI(2.0e-7);
        leftMotorBRPM.setD(3.0);
        leftMotorBRPM.setFF(1.08e-4);
        leftMotorBRPM.setIZone(1000);
        leftMotorBRPM.setDFilter(0);
        leftMotorBRPM.setOutputRange(0, 1);

        rightMotorARPM.setP(5.0e-4);
        rightMotorARPM.setI(5.0e-7);
        rightMotorARPM.setD(5.0e-1);
        rightMotorARPM.setFF(1.7e-4);
        rightMotorARPM.getIZone(500);
        rightMotorARPM.getDFilter(0);

        rightMotorBRPM.setP(5.0e-4);
        rightMotorBRPM.setI(5.0e-7);
        rightMotorBRPM.setD(5.0e-1);
        rightMotorBRPM.setFF(1.7e-4);
        rightMotorBRPM.getIZone(500);
        rightMotorBRPM.getDFilter(0);

    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////Implementation
    @Override
    public double getMaxMeterPerSec() {
        return 14.5; //TODO: verify this
    }
    @Override
    public double getMaxRadPerSec(){
        return 18; //TODO: idk if this even matters
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
////////////////////////////////////////////////////////////////////////////////navx commands
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
        return GenericRobot.super.encoderLeftADriveTicksPerInch();
    }

    @Override
    public double encoderLeftBDriveTicksPerInch() {
        return GenericRobot.super.encoderLeftBDriveTicksPerInch();
    }

    @Override
    public double encoderRightADriveTicksPerInch() {
        return GenericRobot.super.encoderRightADriveTicksPerInch();
    }

    @Override
    public double encoderRightBDriveTicksPerInch() {
        return GenericRobot.super.encoderRightBDriveTicksPerInch();
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
        leftMotorARPM.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setLeftDriveBRPM(double rpm) {
        leftMotorBRPM.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setRightDriveARPM(double rpm) {
        rightMotorARPM.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setRightDriveBRPM(double rpm) {
        rightMotorBRPM.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////Pivot commands
    @Override
    public void resetPIDPivot() {
        PivotMotorPIDLeftA.reset();
        PivotMotorPIDLeftB.reset();
        PivotMotorPIDRightA.reset();
        PivotMotorPIDRightB.reset();
    }
//////////////////////////////////////////////////////////////////////////////pivot encoders

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
    @Override
    public void setPivotLeftMotorA(double Pivot) {
        double power = PivotMotorPIDLeftA.calculate(-Pivot + getPivotLeftMotorA());
        pivotLeftMotorA.set(power);
    }

    @Override
    public void setPivotLeftMotorB(double Pivot) {
        double power = PivotMotorPIDLeftB.calculate(-Pivot + getPivotLeftMotorB());
        pivotLeftMotorB.set(power);
    }

    @Override
    public void setPivotRightMotorA(double Pivot) {
        double power = PivotMotorPIDRightA.calculate(-Pivot + getPivotRightMotorA());
        pivotRightMotorA.set(power);
    }

    @Override
    public void setPivotRightMotorB(double Pivot) {
        double power = PivotMotorPIDRightB.calculate(-Pivot + getPivotRightMotorB());
        pivotRightMotorB.set(power);
    }


}
