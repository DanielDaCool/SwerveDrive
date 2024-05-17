// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Chassis;

public class test extends CommandBase {
  Chassis chassis;
  Translation2d p1;
  Translation2d p2;
  Translation2d handle;
  double time;
  public test(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);

    p1 = new Translation2d(0, 0);
    p2 = new Translation2d(2, 2);
    handle = new Translation2d(-0.5, 2);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 0;

  }


  private Translation2d getCurrentVector(double time){
    Translation2d vec0 = p1.interpolate(handle, time);
    Translation2d vec1 = handle.interpolate(p1, time);
    Translation2d b = vec0.interpolate(vec1, time);
    Translation2d finalVec = b.minus(chassis.getPose().getTranslation());
    return finalVec;

  }
  @Override
  public void execute() {
    Translation2d curVec = getCurrentVector(time);
    chassis.setVelocity(new ChassisSpeeds(curVec.getX(), curVec.getY(), 0));


    time += 0.02;


  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }


  @Override
  public boolean isFinished() {
    return chassis.getPose().getTranslation().getDistance(p2) < 0.1;
  }
}
