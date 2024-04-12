// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class test extends CommandBase {
  Timer time = new Timer();
  public test() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("finished test command");
  }


  @Override
  public boolean isFinished() {
    return time.get() > 5;
  }
}
