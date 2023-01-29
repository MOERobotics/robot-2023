// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class MoeNetVision {

    NetworkTableEntry poseEntry;

    double offset = 0;

    public MoeNetVision(NetworkTableInstance nt){
        var sd = nt.getTable("SmartDashboard");
        poseEntry = sd.getEntry("pose");
    }
    public Pose3d getPose(){
        var pose = poseEntry.getDoubleArray(new double[0]);
        if (pose.length == 0){
            return new Pose3d(5,5, 5, new Rotation3d(0, 0, offset++));
        }



        double x = pose[0];
        double y = pose[1];
        double z = pose[2];
        double rw = pose[3];
        double rx = pose[4];
        double ry = pose[5];
        double rz = pose[6];

        Quaternion quaternion = new Quaternion(rw, rx, ry, rz);

        var rotation = new Rotation3d(quaternion);
        var pose3d = new Pose3d(x, y, z, rotation);
        return pose3d;
    }
}
