// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class MoeNetVision {

    NetworkTableEntry poseEntry;
    Field2d field = new Field2d();


    public MoeNetVision(NetworkTableInstance nt){
        var sd = nt.getTable("SmartDashboard");
        poseEntry = sd.getEntry("pose");
    }
    public Pose3d getPose(){
        var pose = poseEntry.getDoubleArray(new double[0]);
        if (pose.length == 0){
            return null;
        }

        double x = pose[0];
        double y = pose[1];
        double z = pose[2];
        double yaw = pose[3];
        double pitch = pose[4];
        double roll = pose[5];


        var rotation = new Rotation3d(roll, pitch, yaw);
        var pose3d = new Pose3d(x, y, z, rotation);
        return pose3d;
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
