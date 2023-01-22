package frc.robot.subsystems;
/*
Author: Wesam
Date: 1/13/2020
Purpose: Intake
*/
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import frc.robot.commands.GalacticSearch;
import frc.robot.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.I2C.*;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class DriveBase extends SubsystemBase {
//Declares the Motor Controllers using their CAN Id as parameters
  // public static WPI_TalonSRX left_front = new WPI_TalonSRX(RobotMap.fLEFT);
  // public static WPI_TalonSRX left_back = new WPI_TalonSRX(RobotMap.bLEFT);
  // public static WPI_TalonSRX right_front = new WPI_TalonSRX(RobotMap.fRIGHT);
  // public static WPI_TalonSRX right_back = new WPI_TalonSRX(RobotMap.bRIGHT);
  public Autonomous m_auto;
  public static WPI_TalonSRX left1 = new WPI_TalonSRX(Constants.left1), 
                             right1 = new WPI_TalonSRX(Constants.right);
  public static WPI_VictorSPX left2 = new WPI_VictorSPX(Constants.left), 
                              right2 = new WPI_VictorSPX(Constants.right1);
  
  public static SpeedControllerGroup left = new SpeedControllerGroup(left1, left2);
  public static SpeedControllerGroup right = new SpeedControllerGroup(right1, right2);
  public static DifferentialDrive tankdrive = new DifferentialDrive(left, right);
  //Creates MecanumDrive object that simplifies the complex algorithm into a method
  
  AHRS ahrs; //NavX

  public DriveBase(AHRS gyro){
    ahrs = gyro;
    
    m_auto =  new Autonomous(gyro);
  }

  //Method that takes the x,y, and z axis and makes it drive (this method will be referenced in the command)
  public void drive(double x, double y){
    // if (y == 0 && z == 0)
    // {
    //   try {
    //     /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
    //     /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
    //     /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
    //     ahrs = new AHRS(SPI.Port.kMXP); 
    // } catch (RuntimeException ex ) {  

    // }
    // else
    tankdrive.tankDrive(-x,-y);
    // SmartDashboard.putNumber("Left_Front Drive Encoder", -Autonomous.encoder1.getQuadraturePosition());//OI must be the last object declared
    // SmartDashboard.putNumber("Left_Rear Drive Encoder", Autonomous.encoder2.getQuadraturePosition());
    // SmartDashboard.putNumber("Right_Front Drive Encoder", -Autonomous.encoder3.getQuadraturePosition());
    // SmartDashboard.putNumber("Right_Rear Drive Encoder", Autonomous.encoder4.getQuadraturePosition());
    // SmartDashboard.putNumber("Ultrasonic Distance",Robot.ultrasonic.getValue()*RobotMap.sMeters);
  }


  public void move (double left, double right, double leftspeed, double rightspeed){
    m_auto.resetEncoders();
    int x = 0;
    tankdrive.tankDrive(-leftspeed, -rightspeed);
    while (Math.abs(m_auto.getLeft())  < Math.abs(left) || Math.abs(m_auto.getRight()) < Math.abs(right) ){
      tankdrive.tankDrive(-leftspeed, -rightspeed);
      SmartDashboard.putNumber("X value:", m_auto.getX());
      SmartDashboard.putNumber("V value: ",    m_auto.getV());
      SmartDashboard.putNumber("Left Encoder: ", m_auto.getLeft());
      SmartDashboard.putNumber("Right Encoder: ", m_auto.getRight());
      SmartDashboard.putNumber("Area: ", m_auto.getArea());
    }
    
    tankdrive.tankDrive(0,0);
  }

  public void turnToAngle(double forwardSpeed, double theta){
    double leftTurn = 0, rightTurn = 0, minPower = .1;

    double  desiredEffort = 0.2,
            maxAngle = 45,
            currentAngle = getYaw(),
            desiredAngle = theta,
            maxError = maxAngle - desiredAngle,
            kP = desiredEffort / maxError,
            currentError = desiredAngle - currentAngle,
            rawTurningEffort = kP*currentError,
            turningEffort = rawTurningEffort;

    if(Math.abs(rawTurningEffort) < minPower) turningEffort = minPower * rawTurningEffort/Math.abs(rawTurningEffort); //Set minimum value
    if(Math.abs(rawTurningEffort) < 0.02) turningEffort = 0.0; //Set Deadzone

    if (turningEffort > 0){
      leftTurn = turningEffort;
      rightTurn = -turningEffort;
    }
    else {
      leftTurn = turningEffort;
      rightTurn = -turningEffort;
    }

    SmartDashboard.putNumber("kP: ", kP);
    SmartDashboard.putNumber("turn Effort Raw:", rawTurningEffort);
    SmartDashboard.putNumber("turn Effort Mod:", turningEffort);


    tankdrive.tankDrive(-(forwardSpeed+leftTurn), -(forwardSpeed+rightTurn));

  }

  public void resetYaw(){
    ahrs.reset();
  }
  
  public double getYaw(){
    return ahrs.getYaw();
  }
  public double getVZ(){
    return ahrs.getVelocityZ();
  }
  public double getAZ(){
    return ahrs.getWorldLinearAccelZ();
  }

  public double getLeft(){
    return -left1.getSelectedSensorPosition();
  }
  public double getRight(){
    return right1.getSelectedSensorPosition();
  }
  public double getLeftVel(){
    return -left1.getSelectedSensorVelocity();
  }
  public double getRightVel(){
    return right1.getSelectedSensorVelocity();
  }

  public void adjustDistance(){
    
      // if (Math.abs(((Robot.ultrasonic.getValue())*RobotMap.sMeters))>(RobotMap.sError+RobotMap.sDistance))
      //   if (((((Robot.ultrasonic.getValue())*RobotMap.sMeters))-RobotMap.sDistance)>.3)
      //     Robot.drivebase.teleopDrive(0,-.2,0);
      //   else 
      //   Robot.drivebase.teleopDrive(0,-.125,0);
      // else if (Math.abs(((Robot.ultrasonic.getValue())*RobotMap.sMeters))<(RobotMap.sDistance-RobotMap.sError))
      //   if (((((Robot.ultrasonic.getValue())*RobotMap.sMeters))-RobotMap.sDistance)<-.3)
      //     Robot.drivebase.teleopDrive(0,.2,0);
      //   else 
      //     Robot.drivebase.teleopDrive(0,.125,0);
      // else
      //   Robot.drivebase.teleopDrive(0,0,0);
      
      // Timer.delay(.01);
  }
  // @Override
  // public void initDefaultCommand() {
  //   //Setting default command
  //   setDefaultCommand(new MecDrive());
  // }
}