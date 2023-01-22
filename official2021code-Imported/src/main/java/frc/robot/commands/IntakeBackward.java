// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.*;

import java.lang.module.ModuleDescriptor.Requires;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeBackward extends CommandBase {
  private Intake intake1;

  public IntakeBackward(Intake inTake) {
    intake1 = inTake;
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

 
  public void execute() {
    Robot.intake.setBackward();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
