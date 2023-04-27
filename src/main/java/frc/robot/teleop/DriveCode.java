package frc.robot.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoConeCubeStack;
import frc.robot.commands.autoBalance;
import frc.robot.commands.genericCommand;
import frc.robot.generic.GenericRobot;
import frc.robot.vision.Detection;
import frc.robot.vision.MoeNetVision;
import org.opencv.core.Point;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;


public class DriveCode extends GenericTeleop{

    public static final genericCommand
        balance = new autoBalance(),
        autoStack = new AutoConeCubeStack();
    boolean resetting = false;
    Joystick xboxDriver = new Joystick(1);
    Joystick xboxFuncOp = new Joystick(2);
    Joystick buttonBox = new Joystick(0);
    Timer m_timer = new Timer();
    Timer armTimer = new Timer();
    Timer heartBeat = new Timer();
    double xspd, yspd, turnspd;
    double HeightsDeg[] = new double[] {20, 82, 97};
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

    boolean notSeenObjectYet = true;

    double desiredYaw = 0;
    double desiredArmPos = -5;
    PIDController yawControl = new PIDController(1.0e-1, 0,0);
    PIDController inPlacePID = new PIDController(1e-1, 0*3e-3, 4e-3);
    MoeNetVision vision;
    boolean secondTrip = false;
    boolean firstTrip = false;
    boolean autoMode = false;
    double xPoseOfWall = 0;
    boolean lightsOn = false;
    boolean fieldCentric = true;
    boolean autoCollectStopDriving = false;
    boolean heartbeat = false;


    double x = 0;
    double y = 0;
    double armLength = 40;
    Point startingPos = new Point(0,0);
    Point shelfStationRedLeft = new Point (53.5,240.7-7+1.5);
    Point shelfStationRedRight = new Point(53.5, 284-3+1.5);
    Point shelfStationBlueLeft = new Point (597.5, 279);
    Point shelfStationBlueRight = new Point(596.5, 232.7-5-2);
    Point shelfStation = new Point(0,0);
    Rotation2d startRot = new Rotation2d(0);

    double startX = 0;
    boolean clockTurn = false;
    boolean counterTurn = false;
    boolean clockTurnPartial = false;
    boolean counterTurnPartial = false;
    double oldYaw;

    double pigYaw = 0;
    boolean autoDrive = false;
    boolean coneGrabOn = false;

    double desiredDistanceFromHPStation = 1016; //in mm
    double constant = 2000;
    double tinyConstant = .1;


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

        //robot.resetAttitude();
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
        clockTurn = false;
        counterTurn = false;
        desiredArmPos = robot.getPotDegrees();
        if (robot.getRed()){
            startRot = new Rotation2d(Math.PI);
        }
        else{
            startRot = new Rotation2d(0);
        }
        //if (robot.getRed()) robot.setPigeonYaw(180);
        //if (!robot.getRed()) robot.setPigeonYaw(0);
        armTimer.restart();
        heartBeat.restart();
    }

    @Override
    public void teleopPeriodic(GenericRobot robot) {
        SmartDashboard.putBoolean("BalanceCommand", balanceCommand);
        SmartDashboard.putBoolean("balancecommand init", balanceInit);
        SmartDashboard.putNumber("desiredArmPos", desiredArmPos);

        Pose2d currPose = robot.getPose();
        if (armTimer.get() < .1) desiredArmPos = robot.getPotDegrees();


////////////////////////////////////////////////////////////////////////////////////////////////////////////Send Pose to Dash
        Pose2d robotPose = robot.getPose();
        fieldCentric = true;
        armPower = 0;
        autoDrive = false;
        coneGrabOn = false;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Swerve
        xspd = robot.deadzone(-xboxDriver.getRawAxis(1), .35) * robot.getMaxInchesPerSecond();
        yspd = robot.deadzone(-xboxDriver.getRawAxis(0), .35) * robot.getMaxInchesPerSecond();
        turnspd = robot.deadzone(-xboxDriver.getRawAxis(4), .35) * robot.getMaxRadPerSec();

        /*if(xboxDriver.getRawButton(8)){
            balanceCommand = true;
        }
        else{
            balanceCommand = false;
        }
*/
        if (xboxDriver.getRawButton(5)) { // speed un-boosters
            turnspd /= 2;
        }
        if (xboxDriver.getRawButton(6)) {
            xspd /= 2;
            yspd /= 2;
        }

        if (turnspd != 0){
            clockTurn = false;
            counterTurn = false;
            desiredYaw = robot.getYaw();
        }
        else {
            if (xspd != 0 || yspd != 0 || xboxDriver.getRawButton(3) || xboxDriver.getRawButton(2) || clockTurn || counterTurn) {
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
            if (robot.getRed()) robot.setPigeonYaw(180);
            if (!robot.getRed()) robot.setPigeonYaw(0);
        }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////end swerve code

////////////////////////////////////////////////////////////////////////////////////////////////////Point robot to object

        SmartDashboard.putBoolean("Do you see object yet?", notSeenObjectYet);
        SmartDashboard.putNumber("desiredYaw", desiredYaw);
        /*if (xboxDriver.getRawAxis(3) >.1) {
            SmartDashboard.putNumber("desiredYaw", desiredYaw);

            Detection firstDetection = vision.selectedObjectDetection(Detection.Cargo.CUBE, 0, 0, Double.POSITIVE_INFINITY);
            if ((notSeenObjectYet) && (firstDetection != null) ) {
                var objOffset = firstDetection.location.getTranslation().toTranslation2d()
                        .times(Units.metersToInches(1));
                var targetPosition = objOffset.interpolate(new Translation2d(), 0);
                double yDiff = targetPosition.getY();
                double xDiff = targetPosition.getX();
                desiredYaw = robot.getYaw() - Math.atan2(yDiff, xDiff) * 180 / Math.PI;
                turnspd = inPlacePID.calculate(desiredYaw - robot.getYaw());
                fieldCentric = false;
                notSeenObjectYet = false;
                SmartDashboard.putNumber("visionWeirdPoseX", currPose.transformBy(new Transform2d(targetPosition, new Rotation2d())).getX());
                SmartDashboard.putNumber("visionWeirdPosey", currPose.transformBy(new Transform2d(targetPosition, new Rotation2d())).getY());
                SmartDashboard.putNumber("visionWeirdPoseRot", currPose.transformBy(new Transform2d(targetPosition, new Rotation2d())).getRotation().getDegrees());
            }
            else if (notSeenObjectYet) {
                xspd = turnspd = 0;
            }
            else {
                lightsOn = true;
                fieldCentric = false;
                turnspd = inPlacePID.calculate(desiredYaw - robot.getYaw());
                xspd = 84;
                if (robot.cargoDetected()){
                    autoCollectStopDriving = true;
                }
                if (autoCollectStopDriving){
                    xspd = 0;
                    turnspd = 0;
                }
            }
        }
        else {
            lightsOn = false;
            inPlacePID.reset();
            autoCollectStopDriving = false;
            notSeenObjectYet = true;
        }*/
////////////////////////////////////////////////////////////////////////////////////////rotate 180 clockwise of 180 counterclock

        /*if (counterTurnPartial){
            desiredYaw -= 90;
            counterTurnPartial = false;
        }
        if (clockTurnPartial){
            desiredYaw += 90;
            clockTurnPartial = false;
        }
        if (xboxDriver.getRawAxis(5) > .8){
                desiredYaw = oldYaw - 90;
                counterTurn = true;
                clockTurn = false;
                counterTurnPartial = true;
                clockTurnPartial = false;
        }
        else if (xboxDriver.getRawAxis(5) < -.8){
            desiredYaw = oldYaw + 90;
            counterTurn = false;
            clockTurn = true;
            counterTurnPartial = false;
            clockTurnPartial = true;
        }
        else{
            oldYaw = robot.getYaw();
        }
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////D-Pad controls

        double strafexspd = -1*robot.deadzone(xboxFuncOp.getRawAxis(5), .7);
        double strafeyspd = -1*robot.deadzone(xboxFuncOp.getRawAxis(4), .7);
        if (strafexspd != 0) xspd = Math.signum(strafexspd)*12;
        if (strafeyspd != 0) yspd = Math.signum(strafeyspd)*12;
        ///^Roshik controls :)

        switch (DriveCode.POVDirection.getDirection(xboxDriver.getPOV())) {
            case NORTH:
                xspd = 12;
                break;
            case EAST:
                yspd = -12;
                break;
            case SOUTH:
                xspd = -12;
                break;
            case WEST:
                yspd = 12;
                break;

        }

///////////////////////////////////////////////////////////////////////////////////////Start currentChecker to  pick up from hP

        /*if (xboxDriver.getRawButtonPressed(3)){
            autoStep = 0;
            xPoseOfWall = robotPose.getX();
            shelfStation = new Point(shelfStationBlueLeft.x, shelfStationBlueLeft.y);
            if (robot.getRed()){
                shelfStation = new Point(shelfStationRedLeft.x, shelfStationRedLeft.y);
            }
        }
        if (xboxDriver.getRawButtonPressed(2)){
            autoStep = 0;
            xPoseOfWall = robotPose.getX();
            shelfStation = new Point(shelfStationBlueRight.x, shelfStationBlueRight.y);
            if (robot.getRed()){
                shelfStation = new Point(shelfStationRedRight.x, shelfStationRedRight.y);
            }
        }
        if (xboxDriver.getRawButton(3) || xboxDriver.getRawButton(2)){
            coneGrabOn = true;
            SmartDashboard.putNumber("autoStep", autoStep);
            SmartDashboard.putNumber("startPosX", startingPos.x);
            SmartDashboard.putNumber("startPosY", startingPos.y);
            SmartDashboard.putNumber("desiredPoseShelfX", shelfStation.x);
            SmartDashboard.putNumber("desiredPoseShelfY", shelfStation.y);

            boolean poseNull = true;
            Pose3d visPose = vision.getPose();
            if (visPose.getX() != -1){
                poseNull = false;
                x = visPose.getX();
                if (robot.getRed()){
                    y = visPose.getY() + 9;
                }
                else{
                    y = visPose.getY() - 30; //TODO: check if offset shdnt be off
                }
            }

            startingPos = new Point(x, y);
            double shelfCollectSpeed = 48;
            desiredYaw = robot.getYaw();
            pigYaw = robot.getPigeonYaw();
            if (!robot.getRed()) pigYaw -= 180;
            pigYaw = robot.getPigeonBoundedYaw(pigYaw);
            autoDrive = true;
            switch(autoStep) {
                case 0:
                    xspd = yspd = 0;
                    robot.resetStartDists();
                    robot.resetStartPivots();
                    robot.resetStartHeading();
                    if (!robot.getRed()) startRot = new Rotation2d(Math.PI);
                    if (robot.getRed()) startRot = new Rotation2d(0);
                    robot.setPose(new Pose2d(startingPos.x, startingPos.y, startRot));
                    openGripper = true;

                    desiredArmPos = 82;
                    if (!poseNull && Math.abs(pigYaw) <= 5 && Math.abs(robot.getPotDegrees() - desiredArmPos) <= 10){
                        autoStep++;
                        System.out.println("I found my pose and I am ready to go to step 1");
                        m_timer.restart();
                    }
                    break;
                case 1:
                    double xDiff, yDiff, totDiff;
                    xDiff = shelfStation.x - robotPose.getX();
                    if (!poseNull && m_timer.get() >= .15 && Math.abs(xDiff) >= 21){
                        System.out.println(Double.toString(visPose.getX()) + " "+ Double.toString(visPose.getY()));
                        robot.resetStartDists();
                        robot.resetStartHeading();
                        robot.resetStartPivots();
                        robot.setPose(new Pose2d(startingPos.x, startingPos.y, startRot));
                        m_timer.restart();
                    }
                    xDiff = shelfStation.x - robotPose.getX();
                    yDiff = shelfStation.y - robotPose.getY();
                    totDiff = Math.hypot(xDiff, yDiff);
                    SmartDashboard.putNumber("totalDiff", totDiff);
                    xspd = shelfCollectSpeed * xDiff/totDiff;
                    yspd = shelfCollectSpeed * yDiff/totDiff;

                    if (totDiff <= .5) {
                        xspd = 0;
                        yspd = 0;
                        m_timer.reset();
                        m_timer.start();
                        System.out.println("I am ready to go to step 2");
                        autoStep++;
                    }
                    break;
                case 2:
                    xDiff = shelfStation.x - robotPose.getX();
                    yDiff = shelfStation.y - robotPose.getY();
                    totDiff = Math.hypot(xDiff, yDiff);
                    xspd = yspd = 0;
                    if (totDiff > .5){
                        m_timer.restart();
                        xspd = 12 * xDiff/totDiff;
                        yspd = 12 * yDiff/totDiff;
                    }
                    if (m_timer.get() > .5){
                        xspd = yspd = 0;
                        System.out.println("I am ready to go to step 3");
                        m_timer.restart();
                        autoStep ++;
                    }
                    break;
                case 3:
                    xspd = yspd = 0;
                    openGripper = false;
                    if (m_timer.get() >= 1){
                        desiredArmPos = 88;
                        autoStep ++;
                        System.out.println("I am ready to go to step 4");
                        startX = robotPose.getX();
                    }
                    break;
                case 4:
                    xspd = -shelfCollectSpeed;
                    if (robot.getRed()) xspd *= -1;
                    yspd = 0;
                    if (Math.abs(robotPose.getX() - startX) >= 12){
                        coneGrabOn = false;
                        xspd = yspd = 0;
                    }
                    break;
                case 5:
                    coneGrabOn = false;
                    xspd = yspd = 0;
                    break;
            }
            turnspd = yawControl.calculate(pigYaw);

        }
*/
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
        if (xboxFuncOp.getRawAxis(3) > 0.10 || xboxDriver.getRawAxis(3) > 0.1){ //collect in
            raiseTopRoller = false;
            openGripper = true;
            armPower = -.1;
            desiredArmPos = -5;
            if(robot.cargoInCollector()) secondTrip = true;
            if (robot.cargoDetected()) firstTrip = true;
            collectorRPM = 7500;
            if (firstTrip){
                desiredArmPos = -5;
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
            desiredArmPos = -5; //tuck arm in
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
            robot.resetArmPID();
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
            robot.resetArmPID();
            desiredArmPos = HeightsDeg[height];
            pressed = false;
        }
//////////////////////////////////////////////////////////////////////////////////////////lights on close to hp station-heartbeat
        double ourDist = Math.min(robot.getTOFDistance(), constant);
        //ourDist = desiredDistanceFromHPStation + 0;
        double period = 1/(Math.pow(((ourDist-desiredDistanceFromHPStation)/constant),2) + tinyConstant);
        if (Math.sin(2*Math.PI*period*heartBeat.get()) > 0){
            heartbeat = true;
        }
        else{
            heartbeat = false;
        }
        if (ourDist < desiredDistanceFromHPStation) heartbeat = false;
        if (Math.abs(ourDist - desiredDistanceFromHPStation) <= 1){
            heartbeat = true;
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
            robot.setDrive(xspd, yspd, turnspd, autoDrive, fieldCentric);
        }
        robot.collect(collectorRPM, autoMode);
        robot.raiseTopRoller(raiseTopRoller);
        robot.openGripper(openGripper);
        robot.setLightsOn(lightsOn);
        robot.robotHeartbeat(heartbeat);
        robot.coneGrabInAction(coneGrabOn);
        if (armPower != 0) {
            robot.resetArmPID();
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

    public enum POVDirection {
        NORTH(0),
        NORTHEAST(45),
        EAST(90),
        SOUTHEAST(135),
        SOUTH(180),
        SOUTHWEST(225),
        WEST(270), //best
        NORTHWEST(315),
        NULL(-1);

        private final int angle;

        POVDirection(int angle) {
            this.angle = angle;
        }

        public int getAngle() {
            return angle;
        }

        //Kevin voodoo to turn ints into directions
        public static final Map<Integer, POVDirection> directionMap =
                Arrays.stream(POVDirection.values()).collect(
                        Collectors.toMap(
                                POVDirection::getAngle,
                                Function.identity()
                        )
                );

        public static POVDirection getDirection(int angle) {
            return directionMap.getOrDefault(angle, POVDirection.NULL);
        }
    }
}

