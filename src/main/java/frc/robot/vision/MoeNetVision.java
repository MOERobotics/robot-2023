// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public final class MoeNetVision {
    final double ROTATING_THRESHOLD = 1;
    final double METERS_PER_INCH= 0.0254;
    final double INCHES_PER_METER = 39.3701;


    /**
     *
     * @param initial The intial Pose3d
     * @param scale The scale we adjust the Pose3D by (meters to inches or inches to meters)
     * @return adjusted Pose3d
     */
    private static Pose3d scalePose(Pose3d initial, double scale) {
        Pose3d inches = new Pose3d(
                new Translation3d(scale*initial.getX(), scale*initial.getY(), scale*initial.getZ()),
                initial.getRotation());
        return inches;
    }
    List<NetworkCamera> cameras = List.of(
            new LimelightCamera(),
            new OakCamera()
    );
    GenericRobot robot;
    double currentYaw = 0;
    LinkedList<Pose3d> staticPoses = new LinkedList<>();

    Pose3d initialPose;
    Transform3d autoToFieldSpace = new Transform3d();

    public MoeNetVision(GenericRobot robot){
        this.robot = robot;
    }

    /**
     * In disabledPeriodic we are tracking the intial pose of the robot. We do this by grabbing 100 poses while the
     * robot is steady.
     */
    public void disabledPeriodic(){
        double nextYaw = robot.getPigeonYaw();
        if(Math.abs(nextYaw-currentYaw)>ROTATING_THRESHOLD){
            staticPoses.clear();
        }else{
            EstimatedRobotPose currentPose = getVisionPose();
            if(currentPose != null){
                staticPoses.addLast(currentPose.estimatedPose);
            }

            if(staticPoses.size() > 100){
                staticPoses.removeFirst();
            }
        }

        currentYaw = nextYaw;
        initialPose = getPoseAverage();


        SmartDashboard.putNumber("staticPosesSize", staticPoses.size());

        //We start the transform mapping from the origin(0,0,0) to the initial pose
        if(initialPose != null){
            double[] initialPoseXYZ = new double[] {initialPose.getX(), initialPose.getY(), initialPose.getZ()};

            SmartDashboard.putNumberArray("initialPose", initialPoseXYZ);
            autoToFieldSpace = new Transform3d(
                    new Pose3d(),
                    initialPose
            );
        }
    }

    private Pose3d getPoseAverage() {
        double sumX = 0;
        double sumY = 0;
        double sumZ = 0;

        if(staticPoses.size() ==0) {
            return null;
        }

        for (Pose3d pose: staticPoses) {
            sumX += pose.getX();
            sumY += pose.getY();
            sumZ += pose.getZ();
        }

        Translation3d averageTranslation = new Translation3d(sumX/staticPoses.size(), sumY/staticPoses.size(), sumZ/staticPoses.size());
        //TODO were pretending this works
        Rotation3d averageRotation = staticPoses.getLast().getRotation();

        Pose3d newPose = new Pose3d(averageTranslation, averageRotation);

        return newPose;
    }

    public void genericPeriodic() {
        Pose3d odometryPose = new Pose3d(robot.getPose());
        odometryPose = scalePose(odometryPose, METERS_PER_INCH);
        Pose3d odometryPoseFS = odometryPose.transformBy(autoToFieldSpace);

        //Mapping the odometry Pose to hte odometry pose in field space, shifted by Vision pose
        if(getVisionPose() != null) {
            autoToFieldSpace = new Transform3d(
                    odometryPose,
                    odometryPoseFS.interpolate(getVisionPose().estimatedPose, .04)
            );
        }
    }

    private EstimatedRobotPose getVisionPose(){
        EstimatedRobotPose pose = null;
        for(NetworkCamera camera : cameras){
            EstimatedRobotPose camPose;
            try {
                camPose = camera.getPose();
            } catch (Exception e) {
                e.printStackTrace();
                continue;
            }
            if(camPose != null){
                pose = camPose;
                break;
            }
        }
        return pose;
    }

    /**
     * Get the current estimated pose of the robot in field coordinates after processing swerve and vision info
     * @return Pose3d in meters
     */
    public Pose3d getPose(){
        Pose3d odometryPose = new Pose3d(robot.getPose());
        odometryPose = scalePose(odometryPose, METERS_PER_INCH);
        Pose3d odometryPoseFS = odometryPose.transformBy(autoToFieldSpace);
        return odometryPoseFS;
    }

    public boolean poseFound(){
        return getPose() == null;
    }

    /**
     *
     * @return
     */
    public Pose3d robotFieldPoseInches() {
        var pose = getPose();

        if (pose ==null){
            return null;
        }
        return scalePose(pose, INCHES_PER_METER);
    }

    /**
     * Loops through all the cameras to find detections. Returns the first detection found.
     * @return Detection of object
     */
    public Detection firstObjectDetection(){
        for(NetworkCamera camera : cameras){
            //I was upsetty spagetti when I named this variable and I am not sorry
            List<Detection> klad = camera.getDetections();
            if(klad != null){
                for (Detection detection : klad){
                    if (detection.objectType == Detection.Cargo.CONE_BOTTOM)
                        return klad.get(0);
                }
            }
        }
        return null;
    }

    /**
     * Program selects an object type and location to search for. Loops through all the
     * cameras to find detections. This function uses vision to return the closest Detection
     * of that object type.
     *
     * @param type Object type being looked for
     * @param xSelected The X location of the selected object
     * @param ySelected The Y location of the selected Object
     * @param radius The tolerated distance error from the estimated location
     * @return Detection that is closest to the object that is being looked for
     */
    public Detection selectedObjectDetection(Detection.Cargo type, double xSelected, double ySelected, double radius){
        Detection closestDetection = null;
        double closestDistance = radius;

        for(NetworkCamera camera : cameras){
            //I was upsetty spagetti when I named this variable and I am not sorry
            List<Detection> klad;
            try {
                klad = camera.getDetections();
            } catch (Exception e) {
                e.printStackTrace();
                continue;
            }
            if(klad != null){
                for (Detection detection : klad){
                    if (detection.objectType == type){
                        double detectedX = detection.location.getX();
                        double detectedY = detection.location.getY();

                        double distance = Math.sqrt(
                                (Math.abs(detectedX- xSelected) * Math.abs(detectedX- xSelected))
                                +(Math.abs(detectedY- ySelected) * Math.abs(detectedY - ySelected))
                        );

                        if(distance < closestDistance){
                            closestDistance = distance;
                            closestDetection = detection;
                        }

                    }
                }
            }
        }

        return closestDetection;
    }

}
