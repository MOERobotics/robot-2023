package frc.robot.helpers;

import org.opencv.core.Point;

public class AutoCodeLines {


    public static double getDistance(Point p, Point q){
        return Math.sqrt((q.x-p.x)*(q.x-p.x) + (q.y-p.y)*(q.y-p.y));
    }

    public static double getSlopeX(Point p, Point q){
        return (q.x - p.x)/getDistance(p,q);
    }

    public static double getSlopeY(Point p, Point q){
        return (q.y - p.y)/getDistance(p,q);
    }

    public static double getPositionX(Point p, Point q, double s){
        return (getSlopeX(p,q)*(s) + p.x);
    }

    public static double getPositionY(Point p, Point q, double s){
        return (getSlopeY(p,q)*(s) + p.y);
    }

    public static double getVelocityX(Point p, Point q, double s){
        return getSlopeX(p,q);
    }

    public static double getVelocityY(Point p, Point q, double s){
        return getSlopeY(p,q);
    }

    public static double getdS(double distance, double timeToMax, double averageSpeed, double t){
        double time = distance/averageSpeed;
        double maxSpeed = averageSpeed/(time-timeToMax);
        if (t <= timeToMax){
            return getPositionY(new Point(0,0), new Point(timeToMax, maxSpeed), t);
        }
        else if (t <= time-timeToMax){
            return maxSpeed;
        }
        else{
            return getPositionY(new Point(time-timeToMax, maxSpeed), new Point(time, 0), t-(time-timeToMax));
        }
    }

    public static double getS(double distance, double timeToMax, double averageSpeed, double t){
        double time = distance/averageSpeed;
        double maxSpeed = averageSpeed/(time-timeToMax);
        if (t <= timeToMax){
            return getPositionY(new Point(0,0), new Point(timeToMax, maxSpeed), t)*t/2;
        }
        else if (t <= time-timeToMax){
            return timeToMax*maxSpeed/2 + maxSpeed*(t-timeToMax);
        }
        else{
            return timeToMax*maxSpeed + maxSpeed * (time-2*timeToMax)
                    - getPositionY(new Point(time-timeToMax, maxSpeed), new Point(time, 0), t-(time-timeToMax))*(time-t)/2;
        }
    }
}
