// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ShoulderDownPresetCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.CloseHandCommand;
import frc.robot.commands.OpenHandCommand;
import frc.robot.commands.ShoulderDownCommand;
import frc.robot.commands.ShoulderUpCommand;
import frc.robot.commands.TurnTableCommand;
import frc.robot.commands.WristTurnLeftCommand;
import frc.robot.commands.WristTurnRightCommand;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.HandSub;
import frc.robot.subsystems.ShoulderSub;
import frc.robot.subsystems.TurnTableSub;
import frc.robot.subsystems.WristSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  public static ArmSub mArmSub = new ArmSub();

  public static HandSub mHandSub = new HandSub();

  public static ShoulderSub mShoulderSub = new ShoulderSub();

  public static WristSub mWristSub = new WristSub();

  public static TurnTableSub mTurnTableSub = new TurnTableSub();

  public static PS4Controller operator = new PS4Controller(Constants.operator);

  // Turn Hand Buttons 
  JoystickButton handOpen = new JoystickButton(operator, 7);
  JoystickButton handClose = new JoystickButton(operator, 8);

  // Hand Buttons
  JoystickButton WristLeft = new JoystickButton(operator, 5);
  JoystickButton WristRight = new JoystickButton(operator, 6);

  // Shoulder Buttons
  JoystickButton ShoulderUp = new JoystickButton(operator, 4);
  JoystickButton ShoulderDown = new JoystickButton(operator, 2);
  JoystickButton ShoulderDownPreset = new JoystickButton(operator, 3);

/*
 * POV is the D-Pad, Example: POVButton handOpen = new POVButton(operator, 90);
 * The POV buttons are referred to by the angle. Up is 0, right is 90, down is 180, and left is 270.
 * buttonNumber 1 is Square on a PS4 Controller
 * buttonNumber 2 is X on a PS4 Controller
 * buttonNumber 3 is Circle on a PS4 Controller
 * buttonNumber 4 is Triangle on a PS4 Controller
 * buttonNumber 5 is L1 on a PS4 Controller
 * buttonNumber 6 is R1 on a PS4 Controller
 * buttonNumber 7 is L2 on a PS4 Controller
 * buttonNumber 8 is R2 on a PS4 Controller
 * buttonNumber 9 is SHARE on a PS4 Controller
 * buttonNumber 10 is OPTIONS on a PS4 Controller
 * buttonNumber 11 is L3 on a PS4 Controller
 * buttonNumber 12 is R3 on a PS4 Controller
 * buttonNumber 13 is the PlayStaion Button on a PS4 Controller
 * buttonNumber 14 is the Touchpad on a PS4 Controller
 * 
 * https://www.chiefdelphi.com/t/make-motor-move-at-a-specific-rpm/396774
 * Click this link if your trying to send a motor to a certain position
 * 
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/index.html
 * Click this link if your trying to send a motor to a certain position
 */

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    mArmSub.setDefaultCommand(new ArmCommand());
    mTurnTableSub.setDefaultCommand(new TurnTableCommand());
    configureButtonBindings();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    handOpen.whileTrue(new OpenHandCommand());
    handClose.whileTrue(new CloseHandCommand());
    WristLeft.whileTrue(new WristTurnLeftCommand());
    WristRight.whileTrue(new WristTurnRightCommand());
    ShoulderUp.whileTrue(new ShoulderUpCommand());
    ShoulderDown.whileTrue(new ShoulderDownCommand());
    ShoulderDownPreset.onTrue(new ShoulderDownPresetCommand());
  
 

  //https://www.chiefdelphi.com/t/pov-button-programming/362195
  //https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html
  //https://first.wpi.edu/wpilib/allwpilib/docs/release/java/deprecated-list.html
  //This has the new updated names for the stuff like whileHeld    


    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
