package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix.motorcontrol.SensorCollection;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class Autonomous extends SubsystemBase {
  private NetworkTable table;

  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry ta1;
  NetworkTableEntry ta2;

  public static SensorCollection encoder_left = new SensorCollection (DriveBase.left1);
  public static SensorCollection encoder_right = new SensorCollection (DriveBase.right1);
  
  public SerialPort m_arduinoPort;
  private double pixy_xVal = -1, pixy_SetPointVal = -1;

  private AHRS ahrs; //NavX

  public Autonomous(AHRS gyro) {
    ahrs = gyro;

    table = NetworkTableInstance.getDefault().getTable("limelight");
    encoder_right.getQuadraturePosition();
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ta1 = table.getEntry("ta1");
    ta2 = table.getEntry("ta2");
  }

  public double readData() {
    try {
      pixy_xVal = Integer.parseInt(m_arduinoPort.readString().substring(m_arduinoPort.readString().indexOf("x") + 2, m_arduinoPort.readString().indexOf("y") -1));
     
      pixy_SetPointVal = Integer.parseInt(m_arduinoPort.readString().substring(m_arduinoPort.readString().indexOf("|") + 1));
    } catch(Exception e) {
      pixy_xVal = -1;
      pixy_SetPointVal = -1;
    }
    return pixy_xVal;
  }

  public double getYaw(){
    return ahrs.getYaw();
  }

  public double getV() {
    double v = tv.getDouble(0.0);
    return v;
  }

  public double getX() {
    double x = tx.getDouble(0.0);
    
    return x;
  }

  public double getY() {
    double y = ty.getDouble(0.0);
    
    return y;
  }

  public double getArea() {
    double area = ta.getDouble(0.0);    
    return area;
  }

  public void resetEncoders (){
    encoder_left.setQuadraturePosition(0, 0);
    encoder_right.setQuadraturePosition(0, 0);
  }

  public double getLeft() {
    return -encoder_left.getQuadraturePosition()/Constants.encoder_ratio;
  }

  public double getRight(){
    return encoder_right.getQuadraturePosition()/Constants.encoder_ratio;
  }
  
  public double getLeftVel() {
    return -encoder_left.getQuadratureVelocity()/Constants.encoder_ratio;
  }

  public double getRightVel(){
    return encoder_right.getQuadratureVelocity()/Constants.encoder_ratio;
  }

  public double getAvgVel(){
    return (getLeftVel() + getRightVel()) / 2.0;
  }
  // public double getAngleY() {
  //   double a2 = ta2.getDouble(0.0);
  //   return table.getEntry("ty").getDouble(0);
  // }

  // public double getArea() {
  //   return table.getEntry("ta").getDouble(0);
  // }

  // public double getSkew() {
  //   return table.getEntry("ts").getDouble(0);
  // }

  // public double getDist() {
  //   return (2.496 - 0.991) / Math.tan((getAngleY() * Math.PI) / 180);
  // }
}