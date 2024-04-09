package frc.robot.PathFollow;

import static frc.robot.PathFollow.PathFollowConstants.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.fasterxml.jackson.databind.deser.impl.ExternalTypeHandler.Builder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.AvoidBannedZone;
import frc.robot.PathFollow.Util.Leg;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.utils.Trapezoid;
import frc.robot.subsystems.Chassis;

public class FollowSegment extends CommandBase {
  Chassis chassis;
  Pose2d chassisPose = new Pose2d();

  double segmentLength;
  double distanceLeft;

  Rotation2d wantedAngle;

  Segment segment;


  double driveVelocity;
  double rotationVelocity;

  double nextVel;
  double wantedVel;
  double accel;

  Trapezoid driveTrapezoid;
  Trapezoid rotationTrapezoid;

  Translation2d vecVel;



  /**
   * Creates a new path follower using the given points.
   * 
   * @param chassis
   * @param points   from blue alliance
   * @param maxAccel the max accel in m/s2 (squared)
   * 
   */



   public FollowSegment(double wantedVel, double nextVel, Segment segment, Rotation2d wantedAngle){
    this.wantedVel = wantedVel;
    this.nextVel = nextVel;
    this.accel = PATH_ACCEL;
    this.segment = segment;
    this.wantedAngle = wantedAngle;
    this.chassis = RobotContainer.robotContainer.chassis;

   }

 
  
  
  @Override
  public void initialize() {


    driveTrapezoid = new Trapezoid(wantedVel, accel, nextVel);
    rotationTrapezoid = new Trapezoid(PATH_ROTATION_MAX_VELOCITY, PATH_ROTATION_ACCEL, 0);

    segmentLength = segment.getLength();
    distanceLeft = segmentLength;


    vecVel = new Translation2d(0, 0);
  }






  @Override
  public void execute() {


    chassisPose = chassis.getPose();
    
    // current velocity vector
    Translation2d currentVelocity = chassis.getVelocity();

    distanceLeft = segmentLength - segment.distancePassed(chassisPose.getTranslation()); // TODO need to fix this problem
    System.out.println("DISTANCE LEFT: " + distanceLeft);
    

    

    //calc drive velocity using trapezoid
    driveVelocity = Math.min(driveTrapezoid.calcVelocity(distanceLeft, currentVelocity.getNorm()), PATH_MAX_VELOCITY);

    //calc rotation velocity based on Trapezoid
    double rotationVelocity = (Math.abs(wantedAngle.minus(chassis.getAngle()).getDegrees()) <= PATH_ANGLE_OFFSET)
      ? 0 : rotationTrapezoid.calcVelocity(chassis.getChassisSpeeds().omegaRadiansPerSecond, wantedAngle.minus(chassis.getAngle()).getRadians());

    //vector of the velocity
    Translation2d velVector = segment.calc(chassisPose.getTranslation(), driveVelocity);

    ChassisSpeeds speed = new ChassisSpeeds(velVector.getX(), velVector.getY(), rotationVelocity); 
    chassis.setVelocity(speed);

  }

  
  @Override
  public boolean isFinished(){
    return distanceLeft <= PATH_DISTANCE_OFFSET;

  }


}