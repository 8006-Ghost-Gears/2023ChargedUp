// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveDrive.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.SwerveModule.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveModulePosition;
import frc.robot.Constants.SwerveDrive.SWERVE_MODULE_POSITION;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.utils.*;
import frc.robot.utils.RevUtils.PID_SLOT;

public class SwerveModule extends SubsystemBase {

private final SWERVE_MODULE_POSITION m_modulePosition;
private final int m_moduleNumber;
  CANSparkMax m_turnMotor;
  CANSparkMax m_driveMotor;
  private final SparkMaxPIDController m_driveController;
  private SparkMaxPIDController m_turnController;
  public final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;
  CANCoder m_CANCoder;
  double m_angleOffset;
  double m_currentAngle;
  double m_lastAngle;
  boolean m_initSuccess = false;
  Pose2d m_pose;
  SwerveModuleState m_desiredState = new SwerveModuleState();

  SimpleMotorFeedforward feedforward =
          new SimpleMotorFeedforward(
                  ksDriveVoltSecondsPerMeter,
                  kaDriveVoltSecondsSquaredPerMeter,
                  kvDriveVoltSecondsSquaredPerMeter);

  private final ProfiledPIDController m_turningPIDController
          = new ProfiledPIDController(1, 0, 0,
          new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI));

  public SwerveModule(
        SWERVE_MODULE_POSITION modulePosition,
      CANSparkMax turnMotor,
      CANSparkMax driveMotor,
      CANCoder angleEncoder,
      double angleOffset) {
        m_modulePosition = modulePosition;
        m_moduleNumber = m_modulePosition.ordinal();
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    m_CANCoder = angleEncoder;
    m_angleOffset = angleOffset;

    m_driveMotor.restoreFactoryDefaults();
    RevUtils.setDriveMotorConfig(m_driveMotor);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_driveMotor.setInverted(false);

    m_turnMotor.restoreFactoryDefaults();
    RevUtils.setTurnMotorConfig(m_turnMotor);
    m_turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_turnMotor.setInverted(true);

    m_CANCoder.configFactoryDefault();
    m_CANCoder.configAllSettings(CtreUtils.generateCanCoderConfig());

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(kDriveRevToMeters);
    m_driveEncoder.setVelocityConversionFactor(kDriveRpmToMetersPerSecond);

    m_turnEncoder = m_turnMotor.getEncoder();
    m_turnEncoder.setPositionConversionFactor(kTurnRotationsToDegrees);
    m_turnEncoder.setVelocityConversionFactor(kTurnRotationsToDegrees / 60.0);

    m_driveController = m_driveMotor.getPIDController();
    m_turnController = m_turnMotor.getPIDController();

    m_lastAngle = getHeadingDegrees();

    DataLog m_log = DataLogManager.getLog();
  }
   
  private void initCanCoder() {
        Timer.delay(1);
        m_CANCoder.configFactoryDefault();
        m_CANCoder.configAllSettings(CtreUtils.generateCanCoderConfig());
        m_initSuccess = true;
      }

  public boolean getInitSuccess() {
        return m_initSuccess;
      }

      public SWERVE_MODULE_POSITION getModulePosition() {
        return m_modulePosition;
      }

      public int getModuleNumber() {
        return m_moduleNumber;
      }

    
      public void resetAngleToAbsolute() {
        double angle = getCANCoderHeadingDegrees() - m_angleOffset;
        m_turnEncoder.setPosition(angle);
      }
    
      public void resetDistance() {
        m_driveEncoder.setPosition(0);
      }
    
      public double getHeadingDegrees() {
        if(RobotBase.isReal())
          return m_turnEncoder.getPosition();
        else
          return m_currentAngle;
      }
    
      public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
      }
    
      public double getVelocityMetersPerSecond() {
        return m_driveEncoder.getVelocity();
      }
    
      public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = RevUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeedMetersPerSecond;
      m_driveMotor.set(percentOutput);
    } else {
      PID_SLOT DRIVE_PID_SLOT = RobotBase.isReal() ? PID_SLOT.VEL_SLOT : PID_SLOT.SIM_SLOT;
      m_driveController.setReference(
              desiredState.speedMetersPerSecond,
              CANSparkMax.ControlType.kVelocity,
              DRIVE_PID_SLOT.ordinal()
      );
    }

    double angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxSpeedMetersPerSecond * 0.01))
                    ? m_lastAngle
                    : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents Jittering.
    if(RobotBase.isReal())
      m_turnController.setReference(angle, CANSparkMax.ControlType.kPosition, PID_SLOT.POS_SLOT.ordinal());
    else
      m_currentAngle = angle;
//      simTurnPosition(angle);

    m_lastAngle = angle;
//    m_currentAngle = m_turnEncoder.getPosition();
  m_desiredState = desiredState;
  }
    
      public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSecond(), getHeadingRotation2d());
      }
    
      public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(m_turnEncoder.getPosition()));
      }
    
      public void setModulePose(Pose2d pose) {
        m_pose = pose;
      }
    
      public Pose2d getModulePose() {
        return m_pose;
      }
    
      public void setDriveNeutralMode(IdleMode m_mode) {
        m_driveMotor.setIdleMode(m_mode);
      }
    
      public void setTurnNeutralMode(IdleMode m_mode) {
        m_turnMotor.setIdleMode(m_mode);
      }
      public double getCANCoderHeadingDegrees() {
        return m_CANCoder.getAbsolutePosition();
      }
    
    
      private void updateSmartDashboard() {
        SmartDashboard.putNumber("Module " + m_moduleNumber + " Angle",getCANCoderHeadingDegrees());
        SmartDashboard.putNumber("Module " + m_moduleNumber + " Internal Angle",getHeadingDegrees());
        SmartDashboard.putNumber("Module " + m_moduleNumber + " Distance",getPosition().distanceMeters);
        SmartDashboard.putNumber("Module " + m_moduleNumber + " Desired Speed", m_desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Module " + m_moduleNumber + " Speed", getState().speedMetersPerSecond);
      }
    
      @Override
      public void periodic() {
        updateSmartDashboard();
      }

  private ShuffleboardTab m_ShuffleboardTab = Shuffleboard.getTab("Swerve");

}
