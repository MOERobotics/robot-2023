// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.List;

public final class MoeNetVision {

    List<NetworkCamera> cameras = List.of(
            new LimelightCamera(),
            new OakCamera()
    );

    public MoeNetVision(NetworkTableInstance nt){
    }

    public Pose3d getPose(){
        Pose3d pose = null;
        for(NetworkCamera camera : cameras){
            if(camera.getPose() != null){
                pose = camera.getPose();
                break;
            }
        }
        return pose;
    }


}
