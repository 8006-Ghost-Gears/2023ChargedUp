// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ModuleMap;
 
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants { 

    public static final class USB {

        public static final int leftJoystick = 0;
        public static final int rightJoystick = 1;
        public static final int xBoxController = 2;
        public static final int testController = 4;
      }

      public static final int operator = 5;


      public static final class CAN {

        public static final int pigeon = 9;
    
        public static final int frontLeftCanCoder = 10;
        public static final int frontRightCanCoder = 11;
        public static final int backLeftCanCoder = 12;
        public static final int backRightCanCoder = 13;
    
        public static final int frontLeftDriveMotor = 20;
        public static final int frontLeftTurnMotor = 21;
        public static final int frontRightDriveMotor = 22;
        public static final int frontRightTurnMotor = 23;
        public static final int backLeftDriveMotor = 24;
        public static final int backLeftTurnMotor = 25;
        public static final int backRightDriveMotor = 26;
        public static final int backRightTurnMotor = 27;
      }

      public enum SwerveDriveModulePosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
      }

      public static final class SwerveDrive {
        public static final double kTrackWidth = Units.inchesToMeters(30);
        public static final double kWheelBase = Units.inchesToMeters(30);

        public static final Map<SWERVE_MODULE_POSITION, Translation2d> kModuleTranslations =
        Map.of(
            SWERVE_MODULE_POSITION.FRONT_LEFT,
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            SWERVE_MODULE_POSITION.FRONT_RIGHT,
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            SWERVE_MODULE_POSITION.BACK_LEFT,
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            SWERVE_MODULE_POSITION.BACK_RIGHT,
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));

        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(14.5);
        public static final double kMaxRotationRadiansPerSecond = Math.PI * 1.5;
        public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 1.5;
    
        public static final double kP_X = 0.2;
        public static final double kD_X = 0;
        public static final double kP_Y = 0.2;
        public static final double kD_Y = 0;
        public static final double kP_Theta = 8;
        public static final double kD_Theta = 0;

      public static final double kP_Translation = 0.6;
      public static final double kI_Translation = 0;
      public static final double kD_Translation = 0;
      public static final double kP_Rotation = 4;
      public static final double kI_Rotation = 0;
      public static final double kD_Rotation = 0.01;
      

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxRotationRadiansPerSecond, kMaxRotationRadiansPerSecondSquared);

    public enum SWERVE_MODULE_POSITION {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }

    public static double kaDriveVoltSecondsSquaredPerMeter;
  }

  public static final class SwerveModule {
    public static final double kDriveMotorGearRatio = 6.75;
    public static final double kTurningMotorGearRatio = 150.0 / 7.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94);
    public static final int kNeoCPR = 42;
    public static final int kCANCoderCPR = 4096; // Figure this out for Neo Motors.

    //public static final double ksVolts = 0.20322;
      //  public static final double kvVoltSecondsPerMeter = 3.2976;
        //public static final double kaVoltSecondsSquaredPerMeter = 0.67542;
      //  public static final double kPDriveVel = 4.569;

    public static final DCMotor kDriveGearbox = DCMotor.getNEO(1);
    public static final DCMotor kTurnGearbox = DCMotor.getNEO(1);


    public static final double kDriveRevToMeters =
            ((kWheelDiameterMeters * Math.PI) / kDriveMotorGearRatio);
    public static final double kDriveRpmToMetersPerSecond =
            kDriveRevToMeters / 60.0;
    public static final double kTurnRotationsToDegrees =
            360.0 / kTurningMotorGearRatio;
    public static final double kTurningCANCoderDistancePerPulse = 360.0 / kCANCoderCPR;

    public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;

    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
  }
}
