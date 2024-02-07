package frc.robot.commands.autos;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetSwerveNeutralMode;
import frc.robot.commands.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;

 public class DriveTest extends SequentialCommandGroup {
   public DriveTest(SwerveAutoBuilder autoBuilder, SwerveDrive swerveDrive, FieldSim fieldSim) {
     SwerveModuleState[] states =
         new SwerveModuleState[] {
           new SwerveModuleState(0.1, new Rotation2d()),
           new SwerveModuleState(0.1, new Rotation2d()),
           new SwerveModuleState(0.1, new Rotation2d()),
           new SwerveModuleState(0.1, new Rotation2d())
         };
     addCommands(
         new SetSwerveOdometry(
             swerveDrive, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)), fieldSim),
         new RunCommand(() -> swerveDrive.setSwerveModuleStatesAuto(states)),
         new SetSwerveNeutralMode(swerveDrive, IdleMode.kBrake)
             .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
   }
 }