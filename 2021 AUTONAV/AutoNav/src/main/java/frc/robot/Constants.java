// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
import edu.wpi.first.wpilibj.kinematics.*;



public final class Constants {
    /** Motor declerations **/
    //Motors
    final public static int left = 0;
    final public static int left1 = 1;
    final public static int right = 2;
    final public static int right1 = 3;
    
    
    //Encoder Conversions
    //4096 is the units per rotation for the Mag Encoder
    public static final double wheel_circumference_meters = 6 * 3.14 * 0.0254;
    public static final double encoderVelocityToMPS = ((1.0/4096.0) * (wheel_circumference_meters)) / 10.0;
    public static final double encoderPositionToMeters = (1.0/4096.0) * (wheel_circumference_meters);



    //Feedforward/Feedback Gains
    public static final double ks = 0.838;
    public static final double kv = 3.09;
    public static final double ka = 0.33;

    public static final double kPDriveVel = 2.1;

    //Differential Drive Kinematics
    public static final double kTrackwidth = 0.5448;
    public static final DifferentialDriveKinematics kDriveKinematics = 
        new DifferentialDriveKinematics(kTrackwidth); 

    //Max Trajectory Velocity/Acceleration
    public static final double kMaxSpeed = 1;
    public static final double kMaxAcceleration = .5;

    //Ramsete Parameters
    public static final double kRamseteB = 1.5;
    public static final double kRamseteZeta = 0.7;

}
