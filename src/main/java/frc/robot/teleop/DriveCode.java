package frc.robot.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoConeCubeStack;
import frc.robot.commands.autoBalance;
import frc.robot.commands.genericCommand;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.ButtonBox;
import frc.robot.vision.Detection;
import frc.robot.vision.MoeNetVision;
import org.opencv.core.Point;



public class DriveCode extends GenericTeleop{

    public static final genericCommand
        balance = new autoBalance(),
        autoStack = new AutoConeCubeStack();
    boolean resetting = false;
    Joystick xboxDriver = new Joystick(1);
    Joystick xboxFuncOp = new Joystick(2);
    Joystick buttonBox = new Joystick(0);
    Timer m_timer = new Timer();
    double xspd, yspd, turnspd;
    double HeightsDeg[] = new double[] {33.4, 84.8, 101.1};
    int autoStep = 0;

    //////////////////////////////////////////////////////////////////////////////////////////////////Arm Code Constants
    double collectorRPM = 0;
    double armPower = 0;
    boolean raiseTopRoller = true;
    boolean openGripper = true;
    boolean pressed = false;

    boolean balanceCommand = false;
    boolean init = false;
    boolean balanceInit  = false;

    double desiredYaw = 0;
    double desiredArmPos = -6;
    PIDController yawControl = new PIDController(8.0e-2, 0,6e-3);
    PIDController inPlacePID = new PIDController(3.0e-2, 3.0e-3, 0);
    MoeNetVision vision;
    boolean secondTrip = false;
    boolean firstTrip = false;
    boolean autoMode = false;
    double xPoseOfWall = 0;
    boolean lightsOn = false;
    boolean fieldCentric = true;



    double x = 0;
    double y = 0;
    double armLength = 40;
    Point startingPos;
    Point shelfStation = new Point (12.87+armLength,240.7);
    Rotation2d startRot = new Rotation2d(0);

    double startX = 0;

    double visStartingX = 92.87;
    double visStartingY = 220.7;


    @Override
    public void teleopInit(GenericRobot robot) {
        balanceInit = false;
        yawControl.enableContinuousInput(-180,180);
        inPlacePID.enableContinuousInput(-180,180);
        vision = new MoeNetVision(robot);
        resetting = false;
        robot.setOffsetLeftA();
        robot.setOffsetLeftB();
        robot.setOffsetRightA();
        robot.setOffsetRightB();

        robot.resetAttitude();
        robot.resetPIDPivot();

        robot.resetStartHeading();

        robot.resetStartDists();

        robot.resetStartPivots();
        robot.setPose();
        desiredYaw = 0;
        firstTrip = false;
        secondTrip = false;
        pressed = false;
        lightsOn = false;
        fieldCentric = true;
        if (robot.getRed()){
            startRot = new Rotation2d(Math.PI);
        }
        else{
            startRot = new Rotation2d(0);
        }
    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {
        SmartDashboard.putBoolean("BalanceCommand", balanceCommand);
        SmartDashboard.putBoolean("balancecommand init", balanceInit);
        SmartDashboard.putNumber("desiredArmPos", desiredArmPos);
        Pose2d currPose = robot.getPose();



////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
        Pose2d robotPose = robot.getPose();
        fieldCentric = true;
        armPower = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Swerve
        xspd = robot.deadzone(-xboxDriver.getRawAxis(1), .35) * robot.getMaxInchesPerSecond();
        yspd = robot.deadzone(-xboxDriver.getRawAxis(0), .35) * robot.getMaxInchesPerSecond();
        turnspd = robot.deadzone(-xboxDriver.getRawAxis(4), .35) * robot.getMaxRadPerSec();

        if(xboxDriver.getRawButton(8)){
            balanceCommand = true;
        }
        else{
            balanceCommand = false;
        }

        if (xboxDriver.getRawButton(5)) { // speed boosters
            turnspd /= 2;
        }
        if (xboxDriver.getRawButton(6)) {
            xspd /= 2;
            yspd /= 2;
        }

        if (turnspd != 0){
            desiredYaw = robot.getYaw();
        }
        else {
            if (xspd != 0 || yspd != 0 || xboxDriver.getRawButton(3)) {
                turnspd = yawControl.calculate(desiredYaw - robot.getYaw());
            }
        }
        if (xboxDriver.getRawButton(1)) { //resetter
            resetting = true;
            Pose2d m_pose = robot.getPose();
            robot.resetAttitude();
            robot.resetPIDPivot();
            robot.resetStartHeading();
            robot.resetStartDists();
            robot.resetStartPivots();
            robot.setPose(m_pose);
            yawControl.reset();
            desiredYaw = 0;
        }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////end swerve code

////////////////////////////////////////////////////////////////////////////////////////////////////Point robot to object

        if (xboxDriver.getRawAxis(3) >.1) {
            SmartDashboard.putNumber("desiredYaw", desiredYaw);
            Detection firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
            if (firstDetection != null) {
                var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                        .times(Units.metersToInches(1));
                double distance = objOffset.getNorm();
                var targetPosition = objOffset.interpolate(new Translation2d(), 1 - (distance) / distance);
                //Pose2d desiredPose = robotPose.transformBy(new Transform2d(targetPosition, new Rotation2d()));
                double yDiff = targetPosition.getY();
                double xDiff = targetPosition.getX();
                desiredYaw = robot.getYaw() - Math.atan2(yDiff, xDiff)*180/Math.PI;
                turnspd = inPlacePID.calculate(desiredYaw - robot.getYaw());
                fieldCentric = false;
            }
        }

///////////////////////////////////////////////////////////////////////////////////////Start currentChecker to  pick up from hP

        if (xboxDriver.getRawButtonPressed(3)){
            autoStep = 0;
            xPoseOfWall = robotPose.getX();
        }
        if (xboxDriver.getRawButton(3)){
            SmartDashboard.putNumber("autoStep", autoStep);


            x = visStartingX;
            y = visStartingY;
            startingPos = new Point(x, y);
            double shelfCollectSpeed = 48;

            switch(autoStep) {
                case 0:
                    robot.resetStartDists();
                    robot.resetStartPivots();
                    robot.resetStartHeading();
                    robot.setPose(new Pose2d(startingPos.x, startingPos.y, startRot));
                    openGripper = true;

                    //desiredArmPos = 87;
                    autoStep++;
                    break;
                case 1:
                    double xDiff = shelfStation.x - robotPose.getX();
                    double yDiff = shelfStation.y - robotPose.getY();
                    double totDiff = Math.hypot(xDiff, yDiff);
                    SmartDashboard.putNumber("totalDiff", totDiff);
                    xspd = shelfCollectSpeed * xDiff/totDiff;
                    yspd = shelfCollectSpeed * yDiff/totDiff;
                    if (robot.getRed()){
                        xspd *= -1;
                        yspd *= -1;
                    }
                    if (totDiff <= 2) {
                        xspd = 0;
                        yspd = 0;
                        m_timer.reset();
                        m_timer.start();
                        autoStep++;
                    }
                    break;
                case 2:
                    xspd = yspd = 0;
                    openGripper = false;
                    if (m_timer.get() >= 1){
                     //   desiredArmPos = 90;
                        autoStep ++;
                        startX = robotPose.getX();
                    }
                    break;
                case 3:
                    xspd = -shelfCollectSpeed;
                    yspd = 0;
                    if (robotPose.getX() - startX >= 30){
                        xspd = yspd = 0;
                    }
                    break;
            }
            turnspd = yawControl.calculate(desiredYaw - robot.getYaw());

        }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////start collector top roller
        if (xboxFuncOp.getRawButton(5) ){ //move collector up
            raiseTopRoller = true;
        }
        else if (xboxFuncOp.getRawButton(6)){ //move collector down
            raiseTopRoller = false;
        }
        if (xboxFuncOp.getRawButton(4)){ //collect without concern for sensors
            autoMode = true;
        }
        else{
            autoMode = false;
        }
//////////////////////////////////////////////////////////////////////////////////////////////////collector rolls
        if (xboxFuncOp.getRawAxis(3) > 0.10){ //collect in
            raiseTopRoller = false;
            openGripper = true;
            armPower = -.1;
            desiredArmPos = -4;
            if(robot.cargoInCollector()) secondTrip = true;
            if (robot.cargoDetected()) firstTrip = true;
            collectorRPM = 7500;
            if (firstTrip){
                collectorRPM = 3000;
                desiredArmPos = -4;
                armPower = 0;
            }
            if(secondTrip){
                collectorRPM = 0;

                openGripper = false;
            }
            if (autoMode) {
                openGripper = true;
                collectorRPM = 7500;
            }
        }
        else if (xboxFuncOp.getRawAxis(2) > 0.10){ //collect out
            openGripper = true;
            desiredArmPos = -4; //tuck arm in
            collectorRPM = -7500;
        }
        else{ //no more collecting :(
            secondTrip = false;
            firstTrip = false;
            collectorRPM = 0;
        }
///////////////////////////////////////////////////////////////////////////////////////////////////gripper open/close
        if(xboxFuncOp.getRawButton(2)){ //open gripper
            openGripper = true;
        }
        else if (xboxFuncOp.getRawButton(1)) { //close gripper
            openGripper = false;
        }

        double powerForArm = -.2*robot.deadzone(xboxFuncOp.getRawAxis(1), .2); //moves arm up and down
        if (powerForArm != 0){
            armPower = powerForArm;
        }
        if(armPower != 0){
            desiredArmPos = robot.getPotDegrees();
        }
        if (robot.getPotDegrees() > 0) raiseTopRoller = true; //arm fail-safes to obey the rules

//////////////////////////////////////////////////////////////////////////////////////autoStacking commands
        /*
        if button box buttons pressed, autoStackCommand = true;
        only canceled if driver moves joystick
         */
        int height = heightIndex();
        if (pressed){
            desiredArmPos = HeightsDeg[height];
            pressed = false;
        }
//////////////////////////////////////////////////////////////////////////////////////////lights on hp command
        if (xboxFuncOp.getRawButton(7)){
            lightsOn = false;
        }
        if (xboxFuncOp.getRawButton(8)){
            lightsOn = true;
        }
///////////////////////////////////////////////////////////////////////////////////////////////////////Power setters
        if (balanceCommand){
            raiseTopRoller = true;
            if (!balanceInit){
                balance.init(robot);
                balanceInit = true;
            }
            balance.periodic(robot);
        }
        else {
            SmartDashboard.putNumber("turnspd", turnspd);
            balanceInit = false;
            robot.setDrive(xspd, yspd, turnspd, false, fieldCentric);
        }
        robot.collect(collectorRPM, autoMode);
        robot.raiseTopRoller(raiseTopRoller);
        robot.openGripper(openGripper);
        robot.setLightsOn(lightsOn);
        if (armPower != 0) {
            robot.moveArm(armPower);
        }
        else{
            robot.holdArmPosition(desiredArmPos);
        }
    }


    public int heightIndex(){
        int heightIndex = 0;
        if(buttonBox.getRawButtonPressed(1) || buttonBox.getRawButtonPressed(4) || buttonBox.getRawButtonPressed(7)) {
            heightIndex = 0;
            pressed = true;
        }
        if(buttonBox.getRawButtonPressed(2) || buttonBox.getRawButtonPressed(5) || buttonBox.getRawButtonPressed(8)) {
            heightIndex = 1;
            pressed = true;
        }
        if(buttonBox.getRawButtonPressed(3) || buttonBox.getRawButtonPressed(6) || buttonBox.getRawButtonPressed(9)) {
            heightIndex = 2;
            pressed = true;
        }
        return heightIndex;
    }
}