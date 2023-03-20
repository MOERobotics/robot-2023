package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.helpers.AutoCodeLines;
import frc.robot.vision.Detection;
import frc.robot.vision.MoeNetVision;
import org.opencv.core.Point;

public class ScoreAndStop extends genericAutonomous {
    Point startPosition       = new Point(85,108);
    Point firstScorePosition  = new Point(69, 108);
    Point startPositionBlue       = new Point(85,108);
    Point firstScorePositionBlue  = new Point(69, 108);

    double dist1 = AutoCodeLines.getDistance(startPositionBlue, firstScorePositionBlue);

    double defaultPower = 30.0;
    double xspd, yspd, turnspd;
    double xPos;
    double lengthOfField = 650;
    double centerLine = 295;

    @Override
    public void autonomousInit(GenericRobot robot) {
        xspd = yspd = turnspd = 0;
        autonomousStep = 0;
        robot.setPigeonYaw(0);
        defaultPower = 35;
        startPosition.x        = startPositionBlue.x;
        firstScorePosition.x   = firstScorePositionBlue.x;
        if (robot.getRed()){
            startPosition.x        = lengthOfField - startPositionBlue.x;
            firstScorePosition.x   = lengthOfField - firstScorePositionBlue.x;
            robot.setPigeonYaw(180);
        }
    }
}
