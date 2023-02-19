package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Camera in network tables
 */
public interface NetworkCamera {
    /**
     * Gives a pose. If we have no data from the camera, or we are not receiving new data,
     * we set to null.
     * @return EstimatedRobotPose, a combination of Pose3D, timestamps, and accuracy
     */
    EstimatedRobotPose getPose();
}
