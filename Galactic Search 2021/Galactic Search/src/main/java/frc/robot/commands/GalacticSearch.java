// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GalacticSearch extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveBase drive;
  private final Autonomous auto;
  private final Intake intake;


  private boolean ballLocated = false;
  private double balls = 0;
  private boolean done = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GalacticSearch(DriveBase m_drive, Autonomous m_auto, Intake m_intake) {
    auto = m_auto;
    drive = m_drive;
    intake = m_intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, auto, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    auto.resetEncoders();
    drive.resetYaw();
    ballLocated = false;
    balls = 0;
    done = false;
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double startTime1 = Timer.getFPGATimestamp();
    while(Timer.getFPGATimestamp() < startTime1+0.5){             
        drive.drive(0.9,0.9);
    }
    drive.drive(0,0);
    Timer.delay(1);


    for(int i = 0; i<4; i++){

        SmartDashboard.putNumber("balls:", balls);
        SmartDashboard.putNumber("ball Located:",  ballLocated ? 1 : 0);
        SmartDashboard.putNumber("X value:", auto.getX());
        SmartDashboard.putNumber("Y value:", auto.getY());
        SmartDashboard.putNumber("V value: ",    auto.getV());
        SmartDashboard.putNumber("Area: ", auto.getArea());
        SmartDashboard.putNumber("left: ",    drive.getLeft());
        SmartDashboard.putNumber("right: ", drive.getRight());
        
        ballLocated = (auto.getV() == 1);

        //Look for a ball
        while(!ballLocated){
            SmartDashboard.putNumber("Phase: ", 1);
            ballLocated = (auto.getV() == 1);
            drive.drive(-.6, .6);
        }

        //Ball IS in sight
        while(ballLocated && auto.getV() == 1){
            //Move forward and adjust
            while(auto.getArea() < 13 && auto.getV() == 1){
                SmartDashboard.putNumber("Phase: ", 2);
                drive.drive(0.6+auto.getX()/80, 0.6-auto.getX()/80);
            }

            //If the ball is out of range from the intake
            while(auto.getArea() > 13 && Math.abs(auto.getX()) > 9){
                SmartDashboard.putNumber("Phase: ", 3);
                if (auto.getX() > 0){
                    drive.drive(.65, -.65);
                }
                if (auto.getX() < 0){
                    drive.drive(-.65, .65);
                }
            }

            //Once ball in range, move forward and intake
            if(auto.getArea() > 13 && Math.abs(auto.getX()) <= 9){
                SmartDashboard.putNumber("Phase: ", 4);

                //Reset encoders
                drive.resetEncoders();

                //Move forward while intake is on
                intake.setForward();
                
                //drive.move(50, 50, 0.7, 0.7);

                double startTime = Timer.getFPGATimestamp();
                while(Timer.getFPGATimestamp() < startTime+1){             
                    drive.drive(0.7,0.7);
                }
                
                intake.stop();

                //Add to ball counter and reset
                balls+=1;
                ballLocated = false;
            }

        }

    }
    drive.drive(0, 0);
    int x = 0;
     while (Math.abs(drive.getYaw()) > 20 ){
     SmartDashboard.putNumber("Yaw: ", drive.getYaw());
     drive.drive(.65 * -drive.getYaw()/Math.abs(drive.getYaw()), .65 * drive.getYaw()/Math.abs(drive.getYaw()));
   }
   while(true)
    drive.drive( .9, .9);
  
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
