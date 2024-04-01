
package frc.robot.utils;
import static frc.robot.Constants.*;

public class Trapezoid {
    private double maxVel;
    private double accel;
    private double lastVel;
    private double finishVel;
    private double deltaVelocity;


    public Trapezoid(double maxVel, double accel, double finishVel){
        this.accel = accel;
        this.maxVel = maxVel;
        this.finishVel = finishVel;
        deltaVelocity = accel * 0.02;
    }

    private double distanceToVelWithAccel(double currentVel, double targetVel){
        double time = (targetVel - currentVel) / accel;
        return (currentVel * time) + (0.5 * accel * time * time);
    }
    private double distanceWithoutAccel(double currentVel){
        return currentVel * 0.02;
    }

    private boolean isAbleToDeAccel(double currentVel, double distanceLeft){
        return (currentVel + deltaVelocity >= maxVel)
        ? distanceLeft > distanceWithoutAccel(currentVel)
        : distanceLeft > distanceToVelWithAccel(currentVel, currentVel + deltaVelocity);
    }

    public double calcVelocity(double currentVel, double distanceLeft){
        lastVel = currentVel;
        if(distanceLeft < 0) {
            return -calcVelocity(Math.abs(distanceLeft), currentVel);
        }

        return (isAbleToDeAccel(currentVel, distanceLeft))
        ? Math.min(currentVel + deltaVelocity, maxVel)
        : Math.max(currentVel - deltaVelocity, finishVel);
    }
}
