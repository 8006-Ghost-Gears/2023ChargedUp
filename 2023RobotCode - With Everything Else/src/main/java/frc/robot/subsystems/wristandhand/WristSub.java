// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wristandhand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSub extends SubsystemBase {
  /** Creates a new climberSub. */
  CANSparkMax wrist;
  
  public WristSub() {
    wrist = new CANSparkMax(Constants.wrist,MotorType.kBrushless);
    
    wrist.restoreFactoryDefaults();
    wrist.setInverted(false);
    wrist.getEncoder().setPosition(0);

  }


public void wristLeft(){
  wrist.set(0.05);
}

public void wristRight(){
  wrist.set(-0.05);
}

public void stopWrist(){
  wrist.set(0);
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
    SmartDashboard.putNumber("Wrist", wrist.getEncoder().getPosition());
  }
}
