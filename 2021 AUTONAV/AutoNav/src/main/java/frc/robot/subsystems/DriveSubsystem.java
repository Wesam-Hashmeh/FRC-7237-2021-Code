package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.I2C.Port.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.*;

public class DriveSubsystem extends SubsystemBase {

    // Intitializing Motor Controllers
    private final WPI_TalonSRX  left = new WPI_TalonSRX(Constants.left1), 
                                right = new WPI_TalonSRX(Constants.right);
    private final WPI_VictorSPX left2 = new WPI_VictorSPX(Constants.left), 
                                right2 = new WPI_VictorSPX(Constants.right1);

    //Creating SpeedControllorGroups for both sides
    private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(left, left2);
    private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(right, right2);

    //The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

    //Intializing Encoders
    //Not using SensorCollection, getting encoder value from TalonSRX
    //private final SensorCollection m_leftEncoder = new SensorCollection (left);
    //private final SensorCollection m_rightEncoder = new SensorCollection (right);

    //Intializing Gyro
    private final AHRS gyro = new AHRS(Port.kUSB); 

    //Odometry Class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry; 

    
    /**
     * Creates a new DriveSubstyem
     */
    

    public DriveSubsystem() {
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }

    public void teleopDrive (double left, double right){
        m_drive.tankDrive(-left, -right);

    }

    public void periodic() {
        m_odometry.update(gyro.getRotation2d(), 
                         -left.getSelectedSensorPosition()*Constants.encoderPositionToMeters, 
                         right.getSelectedSensorPosition()*Constants.encoderPositionToMeters);
        SmartDashboard.putNumber("Left: ", left.getSelectedSensorPosition()*Constants.encoderPositionToMeters);
        SmartDashboard.putNumber("Right: ", -right.getSelectedSensorPosition()*Constants.encoderPositionToMeters);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(-left.getSelectedSensorVelocity()*Constants.encoderVelocityToMPS,
                                                right.getSelectedSensorVelocity()*Constants.encoderVelocityToMPS);
    }


    public void resetOdometry(Pose2d pose){
        resetEncoders();
        m_odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }
    
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftGroup.setVoltage(-leftVolts);
        rightGroup.setVoltage(rightVolts);
        m_drive.feed();
    }

    public void resetEncoders() {
        left.setSelectedSensorPosition(0);
        right.setSelectedSensorPosition(0);
    }

    public double getAverageEncoderDistance() {
        return (-left.getSelectedSensorPosition()*Constants.encoderPositionToMeters + 
                right.getSelectedSensorPosition()*Constants.encoderPositionToMeters) / 2.0;
    }

    // public Encoder getLeftEncoder() {
    //     return m_leftEncoder;
    // }

    // public Encoder getRightEncoder() {
    //     return m_rightEncoder;
    // }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getYaw();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }
} 