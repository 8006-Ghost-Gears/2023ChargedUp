// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShoulderSub extends SubsystemBase {
  /** Creates a new climberSub. */
  CANSparkMax shoulder;
  public com.revrobotics.RelativeEncoder RelativeEncoder;
  
  
  public ShoulderSub() {
    shoulder = new CANSparkMax(Constants.shoulder,MotorType.kBrushless);
    
    shoulder.restoreFactoryDefaults();
    //shoulderPIDController = shoulder.getPIDController();
    shoulder.setInverted(false);
    shoulder.getEncoder().setPosition(0);
    System.out.println(RelativeEncoder);
    RelativeEncoder = shoulder.getEncoder();

  }



public void shoulderDown(){
  if(shoulder.getEncoder().getPosition() > -1) {
    shoulder.set(-0.5);
  }
  else {
      RobotContainer.mShoulderSub.stopShoulder();
}
}
public void shoulderUp(){
  shoulder.set(0.5);
}

public void shoulderDownPreset(){
  // Define the target position
  // Set the motor to position mode
  //shoulder.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kCurrent);
  // Wait until the motor reaches the target position
  
     if(shoulder.getEncoder().getPosition() < 40) {
      RobotContainer.mShoulderSub.shoulderUp();
     }    
     else if(shoulder.getEncoder().getPosition() > 40) {
      RobotContainer.mShoulderSub.stopShoulder();
    }
  } 


public void stopShoulder(){
  shoulder.set(0);
}

/*  public void setClimber(){
    if(climberL.getEncoder().getPosition() < 104) {
    climberL.set(1);
    } else {
      climberL.set(0);
    }
    if(climberR.getEncoder().getPosition() < 104) {
      climberR.set(1);
    } else {
      climberR.set(0);
    }
}
*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder", shoulder.getEncoder().getPosition());

  }
}

