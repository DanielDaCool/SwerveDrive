package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PathFollow.PathFollow;
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
    return new PathFollow(new pathPoint[]{ new pathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), 0, 4),
      new pathPoint(new Translation2d(3, 3), Rotation2d.fromDegrees(20), 0.5, 1),

    });
  }
}
