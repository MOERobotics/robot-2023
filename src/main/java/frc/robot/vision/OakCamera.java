package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class OakCamera implements NetworkCamera{
    NetworkTableEntry poseEntry;
    NetworkTableEntry detectionsEntry;
    double[] lastPose;

    OakCamera(){
        var nt = NetworkTableInstance.getDefault();
        var sd = nt.getTable("SmartDashboard");
        poseEntry = sd.getEntry("pose");
        detectionsEntry = sd.getEntry("detections");
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

    public List<Detection> getDetections(){
        var detections = detectionsEntry.getDoubleArray(new double[0]);

        List<Detection> debt = new ArrayList<>();

        for(int i =0; i<detections.length; i+=4){
            var x= detections[i];
            var y= detections[i+1];
            var z= detections[i+2];

            if (Math.abs(x) < 0.001 && Math.abs(y) < 0.001 && Math.abs(z) < 0.001 )
                continue;

            Detection.Cargo cargoType = Detection.Cargo.values()[(int)detections[i+3]];
            //10.5 is our offset for hatboro will need to change
            Detection newDebt = new Detection(z,-x- Units.inchesToMeters(10.5),y,cargoType);
            debt.add(newDebt);
        }

        return debt;
    }


}
