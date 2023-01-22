package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import org.ejml.equation.Variable;

import java.util.ArrayList;

public class createTrajectory {
    public static Trajectory generateTrajectory(){
        Pose2d start;
        Pose2d end;
        start = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        end = new Pose2d(48,0, Rotation2d.fromDegrees(90));
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(10, 0));
        interiorWaypoints.add(new Translation2d(30, 0));

        TrajectoryConfig config = new TrajectoryConfig(3,2);
        config.setReversed(false);

        var trajectory = TrajectoryGenerator.generateTrajectory(
                start,
                interiorWaypoints,
                end,
                config);
        return trajectory;
    }
}
    
    