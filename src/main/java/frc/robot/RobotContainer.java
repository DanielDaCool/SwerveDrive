package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PathFollow.Path;
import frc.robot.PathFollow.Util.pathPoint;

import frc.robot.subsystems.Chassis;

public class RobotContainer {
  public static RobotContainer robotContainer;
  public static CommandXboxController controller;
  public Chassis chassis;


  boolean isRed = false;


  public RobotContainer() {
    robotContainer = this;
    controller = new CommandXboxController(0);
    chassis = new Chassis();


    configureBindings();


  }

  public boolean isRed(){
    return isRed;
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return new Path(new pathPoint[]{
      new pathPoint(0, 0, null, 0, 0),
      new pathPoint(3, 3, Rotation2d.fromDegrees(90), 0.5, 3),
      new pathPoint(6, 0, Rotation2d.fromDegrees(180), 0, 0)
    });
  }
}
