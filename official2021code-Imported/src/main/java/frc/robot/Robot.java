// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static Autonomous auto;
  public static GalacticSearch search;
  public static DriveBase drivebase;
  public static Intake intake;
  private RobotContainer m_robotContainer;
  private Timer time;
  //private PathFollower path;
  
  private AHRS ahrs = new AHRS(Port.kUSB); 
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
     // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
     // autonomous chooser on the dashboard.
     m_robotContainer = new RobotContainer(ahrs);
     drivebase = new DriveBase(ahrs);
     auto = new Autonomous(ahrs);    
     intake = new Intake();
     search = new GalacticSearch(auto, drivebase, intake);
     time = new Timer();
    // path = new PathFollower(ahrs);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = search;
    

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }
                                                                                   
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    time.reset();
    time.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    XboxController stick = new XboxController(0);
    drivebase.drive(-stick.getY(Hand.kLeft)/2, -stick.getY(Hand.kRight)/2);
    //SmartDashboard.putNumber("Motor Pwr: ", -stick.getY(Hand.kLeft)/2);
    //drivebase.turnToAngle(0.4, 0);

    SmartDashboard.putNumber("Left Encoder: ", drivebase.getLeft());
    SmartDashboard.putNumber("Right Encoder: ", drivebase.getRight());    
    SmartDashboard.putNumber("Left Encoder Vel.: ", drivebase.getLeftVel());
    SmartDashboard.putNumber("Right Encoder Vel.: ", drivebase.getRightVel());    
    SmartDashboard.putNumber("Yaw d: ", drivebase.getYaw());    
    SmartDashboard.putNumber("Yaw a: ", ahrs.getYaw());  
    SmartDashboard.putNumber("Yaw Vel.: ", drivebase.getVZ());
    SmartDashboard.putNumber("Yaw Acc.: ", drivebase.getAZ());
    SmartDashboard.putNumber("Timefpga: ", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("Time get: ", time.get());
    
  }

  @Override
  public void testInit() {
    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("X", auto.getX());
    SmartDashboard.putNumber("Y", auto.getY());    
    SmartDashboard.putNumber("V", auto.getV());
    SmartDashboard.putNumber("Area", auto.getArea());  
    SmartDashboard.putNumber("Yaw: ", drivebase.getYaw());
    
    //SmartDashboard.putNumber("Timefpga: ", Timer.getFPGATimestamp());
    //SmartDashboard.putNumber("Time get: ", time.get());


  }
}
