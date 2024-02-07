// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static frc.robot.Constants.SwerveDrive.kModuleTranslations;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.utils.ModuleMap;

public class FieldSim {
  private final SwerveDrive m_swerveDrive;

  private static Pose2d robotPose;

  private Field2d m_field2d = new Field2d();

  private Pose2d[] m_swerveModulePoses = {
    new Pose2d(), 
    new Pose2d(), 
    new Pose2d(), 
    new Pose2d()
  };

  public FieldSim(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  public void initSim() {}

  public Field2d getField2d() {
    return m_field2d;
  }

  public void setTrajectory(List<PathPlannerTrajectory> trajectories) {
    List<Pose2d> trajectoryPoses = new ArrayList<>();

    for (var trajectory : trajectories) {
      trajectoryPoses.addAll(
          trajectory.getStates().stream()
              .map(state -> state.poseMeters)
              .collect(Collectors.toList()));
    }

    m_field2d.getObject("trajectory").setPoses(trajectoryPoses);
  }

  public void setTrajectory(PathPlannerTrajectory trajectory) {
    m_field2d.getObject("trajectory").setTrajectory(trajectory);
  }

  public void resetRobotPose(Pose2d pose) {
    m_field2d.setRobotPose(pose);
  }


  private void updateRobotPoses() {
    robotPose = m_swerveDrive.getPoseMeters();
    m_field2d.setRobotPose(robotPose);

    if (RobotBase.isSimulation()) {
        m_field2d
            .getObject("Swerve Modules")
            .setPoses(ModuleMap.orderedValues(m_swerveDrive.getModulePoses(), new Pose2d[0]));
      }

    m_field2d.getObject("Swerve Modules").setPoses(m_swerveModulePoses);
  }

  public void periodic() {
    updateRobotPoses();

    if (RobotBase.isSimulation()) simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {}
}