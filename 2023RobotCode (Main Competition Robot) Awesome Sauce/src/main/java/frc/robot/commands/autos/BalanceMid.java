package frc.robot.commands.autos;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

public class BalanceMid extends SequentialCommandGroup {
  public BalanceMid(
      String pathName,
      SwerveAutoBuilder autoBuilder,
      SwerveDrive swerveDrive,
      FieldSim fieldSim) {

    var trajectory =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(9), Units.feetToMeters(9)));

    var autoPath = autoBuilder.fullAuto(trajectory);

    addCommands(
        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(), fieldSim),
        autoPath,
        new AutoBalance(swerveDrive),
        new SetSwerveNeutralMode(swerveDrive, IdleMode.kBrake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}

/*package frc.robot.commands.autos;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

public class DriveForward extends SequentialCommandGroup {
  public DriveForward(
      String pathName,
      SwerveAutoBuilder autoBuilder,
      SwerveDrive swerveDrive,
      FieldSim fieldSim) {

    var trajectory =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(4), Units.feetToMeters(4)));

    var autoPath = autoBuilder.fullAuto(trajectory);

    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
     4   // fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, trajectory),
        autoPath,
        new SetSwerveNeutralMode(swerveDrive, IdleMode.kBrake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, true, false)));
  }
} */