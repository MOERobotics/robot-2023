// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.generic.GenericRobot;

import java.util.LinkedList;
import java.util.List;

public final class MoeNetVision {
    final double ROTATING_THRESHOLD = 1;
    List<NetworkCamera> cameras = List.of(
            new LimelightCamera(),
            new OakCamera()
    );
    NetworkTableEntry poseEntry;
    Field2d field = new Field2d();

    GenericRobot gr;
    double currentYaw = 0;
    LinkedList<Pose3d> staticPoses = new LinkedList<>();

    Pose3d initialPose;
    Transform3d autoToFieldSpace = new Transform3d();

    public MoeNetVision(GenericRobot gr){
        this.gr = gr;
    }

    public void disabledPeriodic(){
        double nextYaw = gr.getPigeonYaw();
        if(Math.abs(nextYaw-currentYaw)>ROTATING_THRESHOLD){
            staticPoses.clear();
        }else{
            Pose3d currentPose = getVisionPose();
            if(currentPose != null){
                staticPoses.addLast(currentPose);
            }

            if(staticPoses.size() > 100){
                staticPoses.removeFirst();
            }
        }

        currentYaw = nextYaw;
        initialPose = getPoseAverage();

        if(initialPose != null){
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

        double sumRotX = 0;
        double sumRotY = 0;
        double sumRotZ = 0;

        if(staticPoses.size() !=0) {
            return null;
        }

        for (Pose3d pose: staticPoses) {
            sumX += pose.getX();
            sumY += pose.getY();
            sumZ += pose.getZ();
            sumRotX += pose.getRotation().getX();
            sumRotY += pose.getRotation().getY();
            sumRotZ += pose.getRotation().getZ();
        }

        Translation3d averageTranslation = new Translation3d(sumX/staticPoses.size(), sumY/staticPoses.size(), sumZ/staticPoses.size());
        //TODO were pretending this works
        Rotation3d averageRotation = staticPoses.getFirst().getRotation();

        Pose3d newPose = new Pose3d(averageTranslation, averageRotation);

        return newPose;
    }

    public void genericPeriodic() {
        Pose3d odometryPose = new Pose3d(gr.getPose());
        Pose3d odometryPoseFS = odometryPose.transformBy(autoToFieldSpace);

        if(getVisionPose() != null) {
            autoToFieldSpace = new Transform3d(
                    odometryPose,
                    odometryPoseFS.interpolate(getVisionPose(), .04)
            );
        }
    }

    private Pose3d getVisionPose(){
        Pose3d pose = null;
        for(NetworkCamera camera : cameras){
            if(camera.getPose() != null){
                pose = camera.getPose();
                break;
            }
        }
        return pose;
    }

    public Pose3d getPose(){
        return initialPose; //TODO fixme
    }

    public Pose2d robotFieldPoseInches(){
        if (poseFound()) {
            var pose = getPose();
            var Pos2d = pose.toPose2d();
            Pose2d inches = new Pose2d(Units.metersToInches(Pos2d.getX()), Units.metersToInches(Pos2d.getY()),
                                        Pos2d.getRotation());
            field.setRobotPose(inches);
            return field.getRobotPose();
        }
        return null;
    }

    public boolean poseFound(){
        var pose = poseEntry.getDoubleArray(new double[0]);
        if (pose.length == 0){
            return false;
        }
        return true;
    }
}
