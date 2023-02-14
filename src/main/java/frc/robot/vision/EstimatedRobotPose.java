package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class EstimatedRobotPose {
    Pose3d estimatedPose;
    double latencyMS;
    double accuracy;
    public EstimatedRobotPose(Pose3d estimatedPose, double latencyMS, double accuracy) {
        this.estimatedPose = estimatedPose;
        this.latencyMS = latencyMS;
        this.accuracy = accuracy;
    }

}
