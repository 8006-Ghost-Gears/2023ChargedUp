// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakesubs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeSub extends SubsystemBase {
  /** Creates a new climberSub. */
 CANSparkMax intake;
  
  public IntakeSub() {
    intake = new CANSparkMax(Constants.intake,MotorType.kBrushless);
    
    intake.restoreFactoryDefaults();
    intake.setInverted(true);
    intake.getEncoder().setPosition(0);

  }



  public void Intake(double speed) {
    // Check if current position is greater than -1
    intake.set(-speed);
  }
  public void Outtake(double speed) {
    intake.set(speed);
  }


/* In this code, we first get the current position of the shoulder encoder and store it in 
the position variable. Then, we check the position and use the WaitThenRunCommand to schedule 
the appropriate action with a 1-second delay. The WaitThenRunCommand is a custom command that 
extends InstantCommand and takes a delay time (in seconds) and a Runnable as parameters. When 
it is scheduled, it waits for the specified delay time and then runs the Runnable. */ 
  

public void stopIntake(){
  intake.set(0); // elijah allen is not the best button boy, but his sister thoooooooo ;) Jonah:)
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake", intake.getEncoder().getPosition());

  }
}

