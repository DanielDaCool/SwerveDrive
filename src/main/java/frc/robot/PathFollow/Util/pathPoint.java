// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * 
 *A point in the path
 *
 */
public class pathPoint extends Pose2d{
    double radius;
    double velocity;


    public pathPoint(Translation2d p, Rotation2d r, double velocity) {
      this(p.getX(),p.getY(),r,0, velocity);
    }
    public pathPoint(Translation2d p, Rotation2d r, double radius, double velocity) {
      this(p.getX(),p.getY(),r,radius, velocity);
    }

    public pathPoint(double x, double y, Rotation2d rotation, double radius, double velocity) {
        super(x,y,rotation);
        this.radius = radius;
        this.velocity = velocity;
      
    }

    
    public double getRadius()
    {
      return radius;
    }
    public void setRadius(double radius)
    {
      this.radius = radius;
    }
    public double getVelocity(){
      return velocity;
    }
    public void setVelocity(double velocity){
      this.velocity = velocity;
    }

    
    @Override
    public String toString(){
      return "x: " + this.getX() + " y: " + this.getY() + " radius: " + this.getRadius() + " velocity: " + this.getVelocity();
    }



}
