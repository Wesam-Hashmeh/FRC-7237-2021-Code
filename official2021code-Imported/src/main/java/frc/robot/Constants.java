// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        /** Motor declerations **/
    //Motors
    final public static int left = 0;
    final public static int left1 = 1;
    final public static int right = 2;
    final public static int right1 = 3;
        
    //Autonomous
    final public static int wheel_radius = 6; //inches
    final public static double wheel_circumference = wheel_radius * Math.PI; //inches
    final public static double encoder_ratio = 212.21;
    final public static double areaRatio = 20;

    final public static double i_Forward = 1;
    final public static double i_Backward = -1;     
    
        
}
