package frc.robot.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoConeCubeStack;
import frc.robot.commands.autoBalance;
import frc.robot.commands.genericCommand;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.ButtonBox;


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
    double desiredPos = -6;
    PIDController yawControl = new PIDController(1.0e-1, 0,0);
    boolean secondTrip = false;
    boolean firstTrip = false;
    boolean autoMode = false;
    double xPoseOfWall = 0;

    @Override
    public void teleopInit(GenericRobot robot) {
        balanceInit = false;
        yawControl.enableContinuousInput(-180,180);

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
    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {
        SmartDashboard.putBoolean("BalanceCommand", balanceCommand);
        SmartDashboard.putBoolean("balancecommand init", balanceInit);
        SmartDashboard.putNumber("desiredArmPos", desiredPos);


////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
        Pose2d robotPose = robot.getPose();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Swerve
        xspd = robot.deadzone(-xboxDriver.getRawAxis(1), .35) * robot.getMaxInchesPerSecond() / 2;
        yspd = robot.deadzone(-xboxDriver.getRawAxis(0), .35) * robot.getMaxInchesPerSecond() / 2;
        turnspd = robot.deadzone(-xboxDriver.getRawAxis(4), .35) * robot.getMaxRadPerSec() / 2;

        if(xboxDriver.getRawButton(8)){
            balanceCommand = true;
        }
        else{
            balanceCommand = false;
        }

        if (xboxDriver.getRawButton(5)) { // speed boosters
            turnspd *= 2;
        }
        if (xboxDriver.getRawButton(6)) {
            xspd *= 2;
            yspd *= 2;
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

///////////////////////////////////////////////////////////////////////////////////////Start currentChecker to  pick up from hP

        if (xboxDriver.getRawButtonPressed(3)){
            autoStep = 0;
            xPoseOfWall = robotPose.getX();
        }
        if (xboxDriver.getRawButton(3)){
            SmartDashboard.putNumber("autoStep", autoStep);
            xspd = yspd = 0;

            switch (autoStep){
                case 0:
                    xspd = -30;
                    yspd = 0;
                    if (Math.abs(xPoseOfWall - robotPose.getX()) >= 50){
                        xspd = yspd = 0;
                        desiredPos = 78;
                        xPoseOfWall = robotPose.getX();
                        autoStep ++;
                    }
                    break;
                case 1:
                    xspd = 0;
                    yspd = 0;
                    if (Math.abs(robot.getPotDegrees() - desiredPos) <= 2){
                        autoStep ++;
                    }
                    break;
                case 2:
                    xspd = 12;
                    if (Math.abs(xPoseOfWall - robotPose.getX()) >= 21){
                        xspd = yspd = 0;
                        openGripper = false;
                        m_timer.reset();
                        m_timer.start();
                        autoStep ++;
                    }
                    break;
                case 3:
                    if(m_timer.get() <= .2){
                        autoStep++;
                        xPoseOfWall = robotPose.getX();
                    }
                    break;
                case 4:
                    xspd = -12;
                    desiredPos = 84;
                    if (Math.abs(xPoseOfWall) - robotPose.getX() >= 10){
                        xspd = yspd = 0;
                    }
                    break;
            }

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
            desiredPos = -4;
            if(robot.cargoInCollector()) secondTrip = true;
            if (robot.cargoDetected()) firstTrip = true;
            collectorRPM = 7500;
            if (firstTrip) collectorRPM = 3000;
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
            desiredPos = -4; //tuck arm in
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

        armPower = -.2*robot.deadzone(xboxFuncOp.getRawAxis(1), .2); //moves arm up and down
        if(armPower != 0){
            desiredPos = robot.getPotDegrees();
        }
        if (robot.getPotDegrees() > 0) raiseTopRoller = true; //arm fail-safes to obey the rules

//////////////////////////////////////////////////////////////////////////////////////autoStacking commands
        /*
        if button box buttons pressed, autoStackCommand = true;
        only canceled if driver moves joystick
         */
        int height = heightIndex();
        if (pressed){
            desiredPos = HeightsDeg[height];
            pressed = false;
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
            init = false;
            robot.setDrive(xspd, yspd, turnspd);
        }
        robot.collect(collectorRPM, autoMode);
        robot.raiseTopRoller(raiseTopRoller);
        robot.openGripper(openGripper);
        if (armPower != 0) {
            robot.moveArm(armPower);
        }
        else{
            robot.holdArmPosition(desiredPos);
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