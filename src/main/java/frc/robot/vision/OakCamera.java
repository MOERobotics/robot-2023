package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class OakCamera implements NetworkCamera{
    NetworkTableEntry poseEntry;

    OakCamera(){
        var nt = NetworkTableInstance.getDefault();
        var sd = nt.getTable("SmartDashboard");
        poseEntry = sd.getEntry("pose");
    }
    @Override
    public Pose3d getPose() {

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
