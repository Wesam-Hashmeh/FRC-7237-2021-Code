// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.*;

import java.lang.module.ModuleDescriptor.Requires;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GalacticSearch extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Autonomous m_auto;
  private final DriveBase m_drivebase;
  private Intake inTake1;
  private final IntakeForward intakef;
  private final IntakeBackward intakeb;
  private final IntakeStop intakes;

  boolean ballLocated = false;
  int v = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GalacticSearch(Autonomous auto, DriveBase drivebase_1, Intake inTake) {
    m_auto = auto;
    m_drivebase = drivebase_1;
    inTake1 = inTake;
    
    intakef = new IntakeForward(inTake1);
    intakeb = new IntakeBackward(inTake1);
    intakes = new IntakeStop(inTake1);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_auto, m_drivebase, inTake1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_auto.resetEncoders();
    ballLocated = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    m_drivebase.resetYaw();
    
    
    while (v < 3){
      SmartDashboard.putNumber("X value:", m_auto.getX());
      SmartDashboard.putNumber("v:", v);
      SmartDashboard.putBoolean("x:", ballLocated);
      SmartDashboard.putNumber("V value: ", m_auto.getV());
      SmartDashboard.putNumber("Left Encoder: ", m_auto.getLeft());
      SmartDashboard.putNumber("Right Encoder: ", m_auto.getRight());
      SmartDashboard.putNumber("Area: ", m_auto.getArea());

    if (m_auto.getV()==0 && ballLocated==false){//{1 if is in sight, 0 if object is not in sight
      
      SmartDashboard.putNumber("v:", v);
      SmartDashboard.putNumber("Phase:", 1); 
      SmartDashboard.putNumber("X value:", m_auto.getX());
      SmartDashboard.putNumber("V value: ",    m_auto.getV());
      SmartDashboard.putNumber("Left Encoder: ", m_auto.getLeft());
      SmartDashboard.putNumber("Right Encoder: ", m_auto.getRight());
      SmartDashboard.putNumber("Area: ", m_auto.getArea());
      m_drivebase.drive(.6, -.6);
      
    }
    // if (v == 3)
    //     v++;
      
    // else if(((m_auto.getX() < -3 || m_auto.getX() >3)|| m_auto.getX() == 0) && x==0 && m_auto.getV() == 1){

    //     //PHASE TWO: WHEN ROBOT IS ADJUSTING ANGLE 
    //     while ((m_auto.getX() < -3 || m_auto.getX() >3)|| m_auto.getX() == 0 && x==0){
    //       SmartDashboard.putNumber("Phase:", 2); 
    //       SmartDashboard.putNumber("X value:", m_auto.getX());
    //       SmartDashboard.putNumber("V value: ",    m_auto.getV());
    //       SmartDashboard.putNumber("Left Encoder: ", m_auto.getLeft());
    //       SmartDashboard.putNumber("Right Encoder: ", m_auto.getRight());
    //       SmartDashboard.putNumber("Area: ", m_auto.getArea());
    //       if(m_auto.getX() > 0){
    //         m_drivebase.drive(.35,-.35);
    //       }
    //       else if(m_auto.getX() < 0){
    //         m_drivebase.drive(-.35, .35);
    //       }
    //     }
     
    //     m_drivebase.drive(0,0);
    //     SmartDashboard.putNumber("X value:", m_auto.getX());
    //     SmartDashboard.putNumber("V value: ",    m_auto.getV());
    //     SmartDashboard.putNumber("Left Encoder: ", m_auto.getLeft());
    //     SmartDashboard.putNumber("Right Encoder: ", m_auto.getRight());
    //     SmartDashboard.putNumber("Area: ", m_auto.getArea());
    //   }
    //PHASE THREE: DRIVES STRAIGHT UNTIL BALL IS RIGHT IN FRONT OF ROBOT  
    else if(m_auto.getV()== 1 && (m_auto.getArea())< 13 && m_auto.getV() == 1 && ballLocated==false){
      while((m_auto.getArea())< 13 && m_auto.getV() == 1 && ballLocated==false){
        SmartDashboard.putNumber("Phase:", 3); 
        SmartDashboard.putNumber("v:", v);
        SmartDashboard.putNumber("X value:", m_auto.getX());
        SmartDashboard.putNumber("V value: ",    m_auto.getV());
        SmartDashboard.putNumber("Left Encoder: ", m_auto.getLeft());
        SmartDashboard.putNumber("Right Encoder: ", m_auto.getRight());
        SmartDashboard.putNumber("Area: ", m_auto.getArea());
        m_drivebase.drive(.5+m_auto.getX()/80,.5-m_auto.getX()/80);
        
        }

        ballLocated = true;
      }
    else if(m_auto.getV()== 1 && m_auto.getArea() > 13){
      
    //PHASE FOUR: MOVES FIXED DISTANCE UNTIL ROBOT REACHES BALL
      m_auto.resetEncoders();
      SmartDashboard.putNumber("Phase:", 4); 
      SmartDashboard.putNumber("v:", v);
      intakef.execute();
      m_drivebase.move(50, 50, .7, .7);      
      //Timer.delay(.5);
      intakes.execute();
      SmartDashboard.putNumber("Phase:", 5); 
      ballLocated = false;

      //PHASE FIVE: SUCC BALL IN
      Timer.delay(.05);
      
      if (ballLocated == true){
        v++;
      }
      ballLocated = false;
    }
  }
  // while (Math.abs(m_drivebase.getYaw()) > 10){
  //   SmartDashboard.putNumber("Yaw: ", m_drivebase.getYaw());
  //   m_drivebase.drive(-.7 * m_drivebase.getYaw()/Math.abs(m_drivebase.getYaw()), .7 * m_drivebase.getYaw()/Math.abs(m_drivebase.getYaw()));
  // }
  m_drivebase.move(50, 50, .7, .7);
  Timer.delay(5);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
