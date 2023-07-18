package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public class Detection {
    /**
     * The object pose in camera-space (should be robot but we are cringe as of 2/26)
     */
    public final Transform3d location;
    /**
     * The object type being detected.
     */
    public final Cargo objectType;

    /**
     *
     * @param x Horizontal positioning relative to camera.
     * @param y Vertical positioning relative to camera.
     * @param z Distance from camera
     * @param detectedObject Object type of detection.
     */
    public Detection(double x, double y, double z, Cargo detectedObject){
        location = new Transform3d(new Translation3d(x,y,z), new Rotation3d());
        objectType = detectedObject;
    }

    /**
     * Enum determining the 3 types of objects that can be detected.
     */
    public enum Cargo{
        CONE_TOP,
        CONE_BOTTOM,
        CUBE
    }
}
