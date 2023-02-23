package frc.robot.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoConeCubeStack;
import frc.robot.commands.autoBalance;
import frc.robot.commands.autoBalanceBackward;
import frc.robot.commands.genericCommand;
import frc.robot.generic.GenericRobot;


public class DriveCode extends GenericTeleop{

    boolean resetting = false;
    Joystick xbox = new Joystick(1);

    //currently this is the joystick
    Joystick xbox2 = new Joystick(0);

    double xspd, yspd, turnspd;

    genericCommand balance = new autoBalance();
    genericCommand balanceBack = new autoBalanceBackward();
    genericCommand autoStack = new AutoConeCubeStack();

    //////////////////////////////////////////////////////////////////////////////////////////////////Arm Code Constants
    double collectorRPM = 0;
    double armPower = 0;
    boolean dropTopRoller = false;
    boolean openGripper = true;

    boolean balanceCommand = false;
    boolean init = false;
    boolean autoStackCommand = false;

    double desiredYaw = 0;
    double desiredPos = -6;
    PIDController yawControl = new PIDController(.5e-1, 0,0);
    double startAngle;
    boolean btnLeft = false;
    boolean btnRight = false;
    boolean autoBalance;
    int count;
    double totalPathLength = 0;
    boolean firstTrip = false;

    @Override
    public void teleopInit(GenericRobot robot) {
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
    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {


////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
        Pose2d robotPose = robot.getPose();

////////////////////////////////////////////////////////////////////////////////////////////////////////////Swerve
        xspd = robot.deadzone(-xbox.getRawAxis(1), .35) * robot.getMaxInchesPerSecond() / 2;
        yspd = robot.deadzone(-xbox.getRawAxis(0), .35) * robot.getMaxInchesPerSecond() / 2;
        turnspd = robot.deadzone(-xbox.getRawAxis(4), .35) * robot.getMaxRadPerSec() / 2;

        if (xspd != 0 || yspd != 0 || turnspd != 0){
            autoStackCommand = false;
        }


        if (xbox.getRawButton(5)) { // speed boosters
            turnspd *= 2;
        }
        if (xbox.getRawButton(6)) {
            xspd *= 2;
            yspd *= 2;
        }

        if (turnspd != 0){
            desiredYaw = robot.getYaw();
        }
        else {
            if (xspd != 0 || yspd != 0) {
                turnspd = yawControl.calculate(desiredYaw - robot.getYaw());
            }
        }

        if (xbox.getRawButton(1)) { //resetter
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////end swerve code
        // Bumpers left 5, right 6

        if (xbox2.getRawButton(5)){ //move collector up
            dropTopRoller = false;
        }
        else if (xbox2.getRawButton(6)){ //move collector down
            dropTopRoller = true;
        }

        // 2 is b, 3 is x

        if (xbox2.getRawButton(3)){ //collect in
            openGripper = true;
            desiredPos = -6;
            if (robot.cargoInCollector()) firstTrip = true;
            collectorRPM = 0;
            if(!firstTrip) collectorRPM = 7500;
        }
        else if (xbox2.getRawButton(2)){ //collect out
            openGripper = true;
            desiredPos = -6;
            collectorRPM = -7500;
        }
        else{ //no more collecting :(
            firstTrip = false;
            collectorRPM = 0;
        }
        //TODO: change the getRawButton to triggers, left trigger barfs the piece out, right trigger


        //currently using Joystick buttons

        //gripper functions currently do not work.
        if(xbox2.getRawButton(13)){ //open gripper?
            openGripper = true;
        }
        else if (xbox2.getRawButton(12)) { //close gripper?
            openGripper = false;
        }

        armPower = -robot.deadzone(xbox2.getRawAxis(1), .2);
        if(armPower != 0) desiredPos = robot.getPotDegrees();

        //////////////////////////////////////////////////////////////////////////////autoStacking commands
        /*
        if button box buttons pressed, autoStackCommand = true;
        only canceled if driver moves joystick
         */

        ///////////////////////////////////////////////////////////////////////////Power setters
        if (balanceCommand){
            if (!init){
                balance.init();
                init = true;
            }
            balance.periodic();
            robot.collect(collectorRPM);
            robot.raiseTopRoller(dropTopRoller);
            robot.moveArm(armPower);
            robot.openGripper(openGripper);
        }
        else if (autoStackCommand){
            if (!init){
                autoStack.init();
                init = true;
            }
            autoStack.periodic();
        }
        else {
            init = false;
            robot.setDrive(xspd, yspd, turnspd);
            robot.collect(collectorRPM);
            robot.raiseTopRoller(dropTopRoller);
            if (armPower != 0) {
                robot.moveArm(armPower);
            }
            else{
                robot.holdArmPosition(desiredPos);
            }
            robot.openGripper(openGripper);
        }
    }
}