// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveDrive.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SwerveDriveModulePosition;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.Constants.SwerveDrive.ModulePosition;


import java.util.HashMap;
import java.util.Map;


public class SwerveDrive extends SubsystemBase {

  private final HashMap<ModulePosition, SwerveModule> m_swerveModules =
      new HashMap<>(
          Map.of(
              ModulePosition.FRONT_LEFT,
                  new SwerveModule(
                      0,
                      new CANSparkMax(CAN.frontLeftTurnMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                      new CANSparkMax(CAN.frontLeftDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                      new CANCoder(CAN.frontLeftCanCoder),
                      274.482),
              ModulePosition.FRONT_RIGHT,
                  new SwerveModule(
                      1,
                      new CANSparkMax(CAN.frontRightTurnMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                      new CANSparkMax(CAN.frontRightDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                      new CANCoder(CAN.frontRightCanCoder),
                      275.977),
              ModulePosition.BACK_LEFT,
                  new SwerveModule(
                      2,
                      new CANSparkMax(CAN.backLeftTurnMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                      new CANSparkMax(CAN.backLeftDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                      new CANCoder(CAN.backLeftCanCoder),
                      24.961),
              ModulePosition.BACK_RIGHT,
                  new SwerveModule(
                      3,
                      new CANSparkMax(CAN.backRightTurnMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                      new CANSparkMax(CAN.backRightDriveMotor, CANSparkMaxLowLevel.MotorType.kBrushless),
                      new CANCoder(CAN.backRightCanCoder),
                      292.061)));

  private final Pigeon2 m_pigeon = new Pigeon2(CAN.pigeon, "rio");
  private boolean Initialize = false;
  
  private final SwerveDrivePoseEstimator m_odometry;
  private double m_simYaw;
  private DoublePublisher swervePitch, swerveRoll, swerveYaw;

  public SwerveDrive() {
    m_pigeon.configFactoryDefault();
    m_pigeon.setYaw(0);
    m_odometry =
        new SwerveDrivePoseEstimator(
            kSwerveKinematics,
            getHeadingRotation2d(),
            getModulePositions(),
            new Pose2d());
            
    initSmartDashboard();
  }

  private ProfiledPIDController m_xController =
      new ProfiledPIDController(kP_X, 0, kD_X, kThetaControllerConstraints);
  private ProfiledPIDController m_yController =
      new ProfiledPIDController(kP_Y, 0, kD_Y, kThetaControllerConstraints);
  private ProfiledPIDController m_turnController =
      new ProfiledPIDController(kP_Theta, 0, kD_Theta, kThetaControllerConstraints);

  public void drive(
      double throttle,
      double strafe,
      double rotation,
      boolean isFieldRelative,
      boolean isOpenLoop) {
    throttle *= kMaxSpeedMetersPerSecond;
    strafe *= kMaxSpeedMetersPerSecond;
    rotation *= kMaxRotationRadiansPerSecond;

    ChassisSpeeds chassisSpeeds =
        isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                throttle, strafe, rotation, getHeadingRotation2d())
            : new ChassisSpeeds(throttle, strafe, rotation);

            SwerveModuleState[] moduleStates = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeedMetersPerSecond);
        
            for (SwerveModule module : m_swerveModules.values())
              module.setDesiredState(moduleStates[module.getModuleNumber()], isOpenLoop);
    }
  
    public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);
  
      for (SwerveModule module : m_swerveModules.values())
        module.setDesiredState(states[module.getModuleNumber()], isOpenLoop);
    }
  
    public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
      var states =
          kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
      setSwerveModuleStates(states, false);
    }

    public void setOdometry(Pose2d pose) {
      m_pigeon.setYaw(pose.getRotation().getDegrees());
      m_odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
    }

    public void resetOdometry(Pose2d pose, SwerveDrive m_swerveDrive) {
      m_swerveDrive.resetGyro();
      m_odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
    }
  
    public double getPitchDegrees() {
      return m_pigeon.getPitch();
    }
  
    public double getRollDegrees() {
      return m_pigeon.getRoll();
    }
  
    public double getHeadingDegrees() {
      return m_pigeon.getYaw();
      // return 0;
    }
  
    public Rotation2d getHeadingRotation2d() {
      return Rotation2d.fromDegrees(getHeadingDegrees());
    }
  
    public Pose2d getPoseMeters() {
      return m_odometry.getEstimatedPosition();
    }

    public SwerveModule getSwerveModule(int moduleNumber) {
      return m_swerveModules.get(ModulePosition.values()[moduleNumber]);
    }

  public boolean getModuleInitStatus() {
    for (ModulePosition i : m_swerveModules.keySet()) {

      if (!m_swerveModules.get(i).getInitSuccess()) {
        return false;
      }
    }
    return true;
  }

  public void setIdleMode(IdleMode kcoast) {
    for (SwerveModule module : m_swerveModules.values()) {
      module.setDriveNeutralMode(kcoast);
      module.setTurnNeutralMode(kcoast);
    }
  }

  public SwerveDrivePoseEstimator getOdometry() {
    return m_odometry;
  }

  public void resetGyro() {
    m_pigeon.setYaw(0);
    m_pigeon.setAccumZAngle(0);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
      m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
      m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
      m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_swerveModules.get(ModulePosition.FRONT_LEFT).getState(),
      m_swerveModules.get(ModulePosition.FRONT_RIGHT).getState(),
      m_swerveModules.get(ModulePosition.BACK_LEFT).getState(),
      m_swerveModules.get(ModulePosition.BACK_RIGHT).getState()
    };
  }

  public void updateOdometry() {
    m_odometry.update(getHeadingRotation2d(), getModulePositions());

    for (SwerveModule module : m_swerveModules.values()) {
      var modulePositionFromChassis =
          kModuleTranslations[module.getModuleNumber()]
              .rotateBy(getHeadingRotation2d())
              .plus(getPoseMeters().getTranslation());
      module.setModulePose(
          new Pose2d(
              modulePositionFromChassis,
              module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
  }

  private void initSmartDashboard() {
    SmartDashboard.putData(this);

    var swerveTab = NetworkTableInstance.getDefault().getTable("Swerve");
    swervePitch = swerveTab.getDoubleTopic("Pitch").publish();
    swerveRoll = swerveTab.getDoubleTopic("Roll").publish();
    swerveYaw = swerveTab.getDoubleTopic("Yaw").publish();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("gyro " + m_pigeon + " heading", getHeadingDegrees());
    SmartDashboard.putBoolean("ModuleInitStatus", Initialize);
    SmartDashboard.putNumber("X Odometry", m_odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Y Odometry", m_odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber(
        "Rotation Odometry", m_odometry.getEstimatedPosition().getRotation().getDegrees());

    swervePitch.set(getPitchDegrees());
    swerveRoll.set(getRollDegrees());
    swerveYaw.set(getHeadingDegrees());

    SmartDashboard.putNumber("Chassis Heading", getHeadingDegrees());
  }

  @Override
  public void periodic() {
    if (Initialize == false) {
      if (getModuleInitStatus()) {
        Initialize = true;
      }
    }
    updateOdometry();
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed = kSwerveKinematics.toChassisSpeeds(getModuleStates());
    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(20);
    m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
    System.out.println(m_pigeon.getYaw());
  }

  public static Object resetOdometry(Pose2d initialPose) {
    return null;
  }

}

  