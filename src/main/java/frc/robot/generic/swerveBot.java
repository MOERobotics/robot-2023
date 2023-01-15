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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
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
    SparkMaxPIDController PivotMotorPIDLeftA = pivotLeftMotorA.getPIDController();
    SparkMaxPIDController PivotMotorPIDRightA = pivotRightMotorA.getPIDController();
    SparkMaxPIDController PivotMotorPIDLeftB = pivotLeftMotorB.getPIDController();
    SparkMaxPIDController PivotMotorPIDRightB = pivotRightMotorB.getPIDController();

    SparkMaxPIDController leftMotorARPM = leftMotorA.getPIDController();
    SparkMaxPIDController leftMotorBRPM = leftMotorB.getPIDController();
    SparkMaxPIDController rightMotorARPM = rightMotorA.getPIDController();
    SparkMaxPIDController rightMotorBRPM = rightMotorB.getPIDController();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////Further Motor Stuff
    public swerveBot(){

        PivotMotorPIDLeftA.setP(0.1);
        PivotMotorPIDLeftB.setP(0.1);
        PivotMotorPIDRightA.setP(0.1);
        PivotMotorPIDRightB.setP(0.1);

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


        leftMotorARPM.setP(5.0e-5);
        leftMotorARPM.setI(0);
        leftMotorARPM.setD(1.0e-4);
        leftMotorARPM.setFF(1.76182e-4);
        leftMotorARPM.setOutputRange(-1,1);

        leftMotorBRPM.setP(5.0e-5);
        leftMotorBRPM.setI(0);
        leftMotorBRPM.setD(1.0e-4);
        leftMotorBRPM.setFF(1.76182e-4);
        leftMotorBRPM.setOutputRange(-1,1);

        rightMotorARPM.setP(5.0e-5);
        rightMotorARPM.setI(0);
        rightMotorARPM.setD(1.0e-4);
        rightMotorARPM.setFF(1.76182e-4);
        rightMotorARPM.setOutputRange(-1,1);

        rightMotorBRPM.setP(5.0e-5);
        rightMotorBRPM.setI(0);
        rightMotorBRPM.setD(1.0e-4);
        rightMotorBRPM.setFF(1.76182e-4);
        rightMotorBRPM.setOutputRange(-1,1);

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

    @Override
    public void SwerveControllerCommand(Trajectory trajectory, Pose2d pose, SwerveDriveKinematics kinematics, PIDController xController,
                                        PIDController yController, ProfiledPIDController thetaController, Rotation2d desiredRotation) {
        HolonomicDriveController controller = new HolonomicDriveController(xController,yController,thetaController);
                 

    }

    @Override
    public Pose2d getPose(double startHeading, double currHeading, double[] startDistances, double[] startPivots, Pose2d startPose) {
        SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
                kinematics(), Rotation2d.fromDegrees(startHeading),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(startDistances[0], Rotation2d.fromDegrees(-startPivots[0])),
                        new SwerveModulePosition(startDistances[1], Rotation2d.fromDegrees(-startPivots[1])),
                        new SwerveModulePosition(startDistances[2], Rotation2d.fromDegrees(-startPivots[2])),
                        new SwerveModulePosition(startDistances[3], Rotation2d.fromDegrees(-startPivots[3]))
                }, startPose);
        return m_odometry.update(Rotation2d.fromDegrees(-currHeading),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(getDriveDistanceInchesLeftA(), Rotation2d.fromDegrees(-getPivotLeftMotorA())),
                        new SwerveModulePosition(getDriveDistanceInchesRightA(), Rotation2d.fromDegrees(-getPivotRightMotorA())),
                        new SwerveModulePosition(getDriveDistanceInchesLeftB(), Rotation2d.fromDegrees(-getPivotLeftMotorB())),
                        new SwerveModulePosition(getDriveDistanceInchesRightB(), Rotation2d.fromDegrees(-getPivotRightMotorB()))
        });
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////Pivot commands
    @Override
    public void resetPIDPivot() {

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
    double rotOverDeg = (150.0/7)/360;
    @Override
    public void setPivotLeftMotorA(double Pivot) {
        PivotMotorPIDLeftA.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setPivotLeftMotorB(double Pivot) {
        PivotMotorPIDLeftB.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setPivotRightMotorA(double Pivot) {
        PivotMotorPIDRightA.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setPivotRightMotorB(double Pivot) {
        PivotMotorPIDRightB.setReference(Pivot*rotOverDeg, CANSparkMax.ControlType.kPosition);
    }


}
