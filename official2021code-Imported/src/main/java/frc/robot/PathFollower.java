
// package frc.robot;

// import edu.wpi.first.wpilibj.*;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.*;

// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.SerialPort.Port;


// public class PathFollower{
    
//     private Timer time = new Timer();
//     private Autonomous auto;
//     private DriveBase drive;


//     public PathFollower(AHRS gyro){
//         auto = new Autonomous(gyro);
//         drive = new DriveBase(gyro);
//     }

//     //Get the derivative at a point
//     private double derivative(PathFormula f, double px){
//         final double dx = 0.000001; // 10 zeros
//         return (f.eval(px+dx) - f.eval(px))/dx;
//     }

//     //Get the angle (in degrees) of the tangent line at a point
//     private double getTangentAngle(PathFormula f, double px){ 
//         return Math.toDegrees(Math.atan(derivative(f,px)));
//     }
//     private double getTangentAngle(double slope){ 
//         return Math.toDegrees(Math.atan(slope));
//     }

//     //Get distance traveled
//     private double getLinearDistance(){
//         return time.get() * auto.getAvgVel();
//     }

//     //Reset time to zero
//     public void resetTime(){
//         time.reset();
//         time.start();
//     }

//     //Follow a path from point a to b at the specified speed
//     public void followPath(PathFormula f, double speed, double a, double b){
//         double  dist = a + getLinearDistance(),
//                 slope = derivative(f,dist),
//                 angle = getTangentAngle(slope);
        
//         while (dist < b){
//             dist = a + getLinearDistance();
//             slope = derivative(f,dist);
//             angle = getTangentAngle(slope);

//             drive.turnToAngle(speed, -angle);
    
//             SmartDashboard.putNumber("Time", time.get());
//             SmartDashboard.putNumber("dist", dist);
//             SmartDashboard.putNumber("slope", slope);
//             SmartDashboard.putNumber("angle", angle);
//         }
        
    
//     }
// }