package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public class Detection {
    public final Transform3d location;
    public final Cargo objectType;

    public Detection(double x, double y, double z, Cargo detectedObject){
        location = new Transform3d(new Translation3d(x,y,z), new Rotation3d());
        objectType = detectedObject;
    }

    public enum Cargo{
        CONE_TOP,
        CONE_BOTTOM,
        CUBE
    }
}
