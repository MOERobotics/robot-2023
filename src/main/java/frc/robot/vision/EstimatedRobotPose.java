package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class EstimatedRobotPose {
    /**
     * A Pose3D of our robot in field space. This data is in meters. If it isn't it will be converted by this point.
     */
    public final Pose3d estimatedPose;
    double latencyMS;

    /**
     * Bigger is better not real units
     */
    double accuracy;
    public EstimatedRobotPose(Pose3d estimatedPose, double latencyMS, double accuracy) {
        this.estimatedPose = estimatedPose;
        this.latencyMS = latencyMS;
        this.accuracy = accuracy;
    }

}
