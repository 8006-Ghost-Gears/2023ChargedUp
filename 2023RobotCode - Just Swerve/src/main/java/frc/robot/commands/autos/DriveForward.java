package frc.robot.commands.autos;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.SetSwerveNeutralMode;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
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
        // fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, trajectory),
        autoPath,
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
import frc.robot.commands.SetSwerveNeutralMode;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

public class DriveForward extends SequentialCommandGroup {
  public DriveForward(
      String pathName, SwerveAutoBuilder autoBuilder, SwerveDrive swerveDrive, FieldSim fieldSim) {

    var trajectory =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(6)));

    var autoPath = autoBuilder.fullAuto(trajectory);

    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
        // fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, trajectory),
        autoPath,
        new SetSwerveNeutralMode(swerveDrive, IdleMode.kBrake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
*/
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 /* 
package frc.robot.commands.autos;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetSwerveNeutralMode;
import frc.robot.commands.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;


/** Add your docs here. */
/* 
public class DriveForward extends SequentialCommandGroup {
  public DriveForward(String pathName, SwerveAutoBuilder autoBuilder, SwerveDrive swerveDrive, FieldSim fieldSim) {

    var trajectory =
        TrajectoryUtils.readTrajectory(
            "DriveForward", new PathConstraints(Units.feetToMeters(2), Units.feetToMeters(2)));

    var autoPath = autoBuilder.fullAuto(trajectory);
    addCommands(
        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(), fieldSim),
        autoPath,
        new SetSwerveNeutralMode(swerveDrive, IdleMode.kBrake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}

*/