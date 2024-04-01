package frc.robot.subsystems.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.simulation.Motor;

import static frc.robot.Constants.*;

public class SwerveModule {
    private Motor moveMotor;
    private Motor angleMotor;

    public SwerveModule() {
        moveMotor = new Motor();
        angleMotor = new Motor();
    }

    /**
     * Forces the module to completely stop
     */
    public void stop() {
        moveMotor.setVelocity(0);
        angleMotor.setVelocity(0);
    }

    /**
     * Sets the move velocity of the module
     * @param v Velocity in m/s
     */
    public void setVelocity(double v) {
        moveMotor.setVelocity(v * PULSES_PER_METER / 10);
    }

    /**
     * Returns the move velocity of the module
     * @return Velocity in m/s
     */
    public double getVelocity() {
        return moveMotor.getVelocity() / PULSES_PER_METER * 10;
    } 

    /**
     * Returns the distance the module drove
     * @return Distance in encoder pulses
     */
    public double getDistance() {
        return moveMotor.getDistance();
    }

    /**
     * Sets the angle of the module
     * @param angle
     */
    public void setAngle(Rotation2d angle) {
        angleMotor.setPosition(calculateTarget(angle.getDegrees()));
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleMotor.getDistance() / PULSES_PER_DEGREE);
    }

    /**
     * Sets the state of the module (drive speed and angle)
     * @param state
     */
    public void setState(SwerveModuleState state) {
        setVelocity(state.speedMetersPerSecond);
        setAngle(state.angle);
    }

    /**
     * Returns the position state of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance() / PULSES_PER_METER, getAngle());
    }

     /**
      * Updates the simulation motors
      */
    public void update() {
        moveMotor.update();
        angleMotor.update();
    }

    public static double getAngleDifference(double current, double target) {
        double difference = (target - current) % 360;
        return difference - ((int)difference / 180) * 360;
    }

    private double calculateTarget(double targetAngle) {
        double difference = getAngleDifference(getAngle().getDegrees(), targetAngle);
        return angleMotor.getDistance() + (difference * PULSES_PER_DEGREE);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }
}
