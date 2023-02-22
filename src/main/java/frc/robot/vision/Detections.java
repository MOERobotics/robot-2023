package frc.robot.vision;

import lombok.AllArgsConstructor;

@AllArgsConstructor
public class Detections {
    double x;
    double y;
    double z;
    Cargo objectType;

    public enum Cargo{
        CONE_TOP,
        CONE_BOTTOM,
        CUBE
    }
}
