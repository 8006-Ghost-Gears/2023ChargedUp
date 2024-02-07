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
public final class Constants {

  public static final int operator = 0;

  public static final int arm = 1;
  public static final int shoulder = 2;
  public static final int hand = 3;
  public static final int wrist = 4;
  public static final int turntable = 5;



  public static final double proportialPIDConstant = 0.0002;
  public static final double integralPIDConstant = 0.0;
  public static final double derivativePIDConstant = 0.0;
  public static final double integralPIDZone = 0.0;
  public static final double armFeedForwardPIDConstant = 0.000170;
  public static final double maxPIDOutput = 1.0;
  public static final double minPIDOutput = 0.0;


}

