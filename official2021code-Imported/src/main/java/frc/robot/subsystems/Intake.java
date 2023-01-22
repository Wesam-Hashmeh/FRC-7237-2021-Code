package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix.motorcontrol.SensorCollection;

public class Intake extends SubsystemBase {
  final Spark intakeMotor = new Spark(3);

    public void setForward(){
      
      intakeMotor.set(Constants.i_Forward);
    }

    public void setBackward() {
      intakeMotor.set(Constants.i_Backward);
    }

    public void stop() {
      intakeMotor.set(0);
    }
  }

  
