// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class MoeNetVision {

    NetworkTableEntry poseEntry;

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
        double pitch = pose[3];
        double roll = pose[4];
        double yaw = pose[5];

        var rotation = new Rotation3d(roll, pitch, yaw);
        var pose3d = new Pose3d(x, y, z, rotation);
        return pose3d;
    }
}
