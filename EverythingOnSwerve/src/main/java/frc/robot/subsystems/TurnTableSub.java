// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class TurnTableSub extends SubsystemBase {
  /** Creates a new TurnTable. */
  CANSparkMax turntable;
  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-phantom");

  public TurnTableSub() {
    turntable = new CANSparkMax(Constants.turntable,MotorType.kBrushless);

    turntable.restoreFactoryDefaults();
    turntable.setInverted(true); // use to be true
    turntable.setOpenLoopRampRate(0.5); // Previosuly all 0.2


  }

    public void setPower(double turntablepower){
      turntable.set(turntablepower);


      //single line comment format "//"
      //mass comment format 
      /*
      */
      }
  

  public void turntotarget(){
    double targetx = limelight.getEntry("tx").getDouble(0);
    if (targetx > 2){
      setPower(0.05);
    
    }
    else if (targetx < -2){
      setPower(0.05);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("limelight", limelight.getEntry("tx").getDouble(0));
    //SmartDashboard.putNumber("Distance", getDistance());
  }
}
