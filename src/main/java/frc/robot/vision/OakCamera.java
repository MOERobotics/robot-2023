package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Arrays;

public class OakCamera implements NetworkCamera{
    NetworkTableEntry poseEntry;
    double[] lastPose;

    OakCamera(){
        var nt = NetworkTableInstance.getDefault();
        var sd = nt.getTable("SmartDashboard");
        poseEntry = sd.getEntry("pose");
    }
    @Override
    public EstimatedRobotPose getPose() {

        var pose = poseEntry.getDoubleArray(new double[0]);
        if (pose.length == 0){
            return null;
        }
        // This checks whether lastPose and pose are equal
        // If they are equal, that means the pose hasn't changed
        if(Arrays.equals(lastPose, pose)) {
            return null;
        }
        lastPose = pose;

        double x = pose[0];
        double y = pose[1];
        double z = pose[2];
        double w = pose[3];
        double i = pose[4];
        double j = pose[5];
        double k = pose[6];
        double latency = 0;
        double accuracy = 1;
        if(pose.length == 9) {
            latency = pose[7];
            accuracy = pose[8];
        }

        var rotation = new Rotation3d(new Quaternion(w,i,j,k));
        var pose3d = new Pose3d(x, y, z, rotation);
        return new EstimatedRobotPose(pose3d, latency, accuracy);
    }
}
