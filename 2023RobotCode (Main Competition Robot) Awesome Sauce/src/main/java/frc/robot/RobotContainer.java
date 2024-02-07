// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.SetSwerveDriveBalance;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.ArmCubeIntake;
import frc.robot.commands.arm.ArmStowed;
import frc.robot.commands.arm.ShoulderDownCommand;
import frc.robot.commands.arm.ShoulderUpCommand;
import frc.robot.commands.autos.BalanceMid;
import frc.robot.commands.autos.BlueStartRightChargeStation;
import frc.robot.commands.autos.DriveBackward;
import frc.robot.commands.autos.DriveForwardLong;
import frc.robot.commands.autos.DriveForwardShort;
import frc.robot.commands.intakecommands.IntakeCommand;
import frc.robot.commands.intakecommands.OuttakeCommand;
import frc.robot.commands.intakecommands.WristCommand;
import frc.robot.commands.intakecommands.WristCommand;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.arm.ArmSub;
import frc.robot.subsystems.arm.ShoulderSub;
import frc.robot.subsystems.intakesubs.IntakeSub;
import frc.robot.subsystems.intakesubs.WristSub;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.utils.ModuleMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DataLog m_logger = DataLogManager.getLog();

  // The robot's subsystems and commands are defined here...

  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive);

  HashMap<String, Command> m_eventMap = new HashMap<>();
  private SwerveAutoBuilder m_autoBuilder;

  static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
  static XboxController xBoxController = new XboxController(Constants.USB.xBoxController);
  static PS4Controller testController = new PS4Controller(Constants.USB.testController);

  public Trigger[] leftButtons = new Trigger[2];
  public Trigger[] rightButtons = new Trigger[2];
  public Trigger[] xBoxButtons = new Trigger[10];
  public Trigger[] xBoxPOVButtons = new Trigger[4];
  public Trigger xBoxLeftTrigger, xBoxRightTrigger;

  // The robot's subsystems and commands are defined here...

  public static ArmSub mArmSub = new ArmSub();

  //public static ConeModeSub mConeModeSub = new ConeModeSub();

  public static WristSub mWristSub = new WristSub();

  public static IntakeSub mIntakeSub = new IntakeSub();

  public static ShoulderSub mShoulderSub = new ShoulderSub();

  public static PS4Controller operator = new PS4Controller(Constants.operator);


  //Auto Stabalize Button 
  JoystickButton Stabalize = new JoystickButton(testController, 5);

  // Intake Buttons
  //POVButton ConeModeSwitch = new POVButton(operator, 90);
  JoystickButton Intake = new JoystickButton(operator, 7);
  JoystickButton Outtake = new JoystickButton(operator, 8);

  // Preset Buttons
  JoystickButton ShoulderUp = new JoystickButton(operator, 4);
  JoystickButton ShoulderDown = new JoystickButton(operator, 2);
  JoystickButton CubeIntake = new JoystickButton(operator, 3);
  //JoystickButton HighScoringCube = new JoystickButton(operator, 4);
  //JoystickButton MidScoringCube = new JoystickButton(operator, 1);
  //JoystickButton LowScoringCube = new JoystickButton(operator, 3);
  JoystickButton Stowed = new JoystickButton(operator, 12);



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
 * 
 * If you ever run into that problem like The Java Server Crashed 5 Times and will not be restarted, (worked for me)
 * then just delete all the WPILIB VS Codes and The Visual Studio Code that is on the pc, this is because Wpilib uses their own
 * VS Code when you install the latest version!
 */

  public RobotContainer() {
    initializeSubsystems();
    initAutoBuilder();
    initializeAutoChooser();
    //initializeAutoChooser();

    // Configure the button bindings
    mWristSub.setDefaultCommand(new WristCommand());
    //mTurnTableSub.setDefaultCommand(new TurnTableCommand());
    configureButtonBindings();
   UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(160, 120); //Usually (640,320)
    camera.setFPS(30);
    PortForwarder.add(5800, "10.80.6.12", 5800);
    PortForwarder.add(5801, "10.80.6.12", 5801);
    PortForwarder.add(5802, "10.80.6.12", 5802);
  }

  
  public void initializeSubsystems() {
    m_swerveDrive.setDefaultCommand(
        new SetSwerveDrive(
            m_swerveDrive,
            () -> -testController.getRawAxis(1),
            () -> -testController.getRawAxis(0),
            () -> -testController.getRawAxis(2)));
    m_fieldSim.initSim();
  }

  //Add negative to change direction of the command since this is the axis that appears in the frc drive station.

/**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    for (int i = 0; i < leftButtons.length; i++)
      leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightButtons.length; i++)
      rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
    for (int i = 0; i < xBoxButtons.length; i++)
      xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
    for (int i = 0; i < xBoxPOVButtons.length; i++)
      xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 90));

    Intake.whileTrue(new IntakeCommand());
    Outtake.whileTrue(new OuttakeCommand());
    ShoulderUp.whileTrue(new ShoulderUpCommand());
    ShoulderDown.whileTrue(new ShoulderDownCommand());
    CubeIntake.onTrue(new ArmCubeIntake());
    Stowed.onTrue(new ArmStowed());
    //ConeModeSwitch.onTrue(new ConeModeCommand());
    Stabalize.whileTrue(new SetSwerveDriveBalance(m_swerveDrive, null, null, null).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));


  }
  
    /* 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
    //return m_autoCommand;
  }

  public void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    //Cero para Rivera

    m_autoChooser.addOption(
       "DriveForwardShort", new DriveForwardShort("DriveForwardShort", m_autoBuilder, m_swerveDrive, m_fieldSim));
    SmartDashboard.putData("Auto Selector", m_autoChooser);

    m_autoChooser.addOption(
       "DriveForwardLong", new DriveForwardLong("DriveForwardLong", m_autoBuilder, m_swerveDrive, m_fieldSim));
    SmartDashboard.putData("Auto Selector", m_autoChooser);
    
    m_autoChooser.addOption(
        "DriveBackward", new DriveBackward("DriveBackward", m_autoBuilder, m_swerveDrive, m_fieldSim));
    SmartDashboard.putData("Auto Selector", m_autoChooser);
    
    m_autoChooser.addOption(
        "BlueStartRightChargeStation", new BlueStartRightChargeStation("BlueStartRightChargeStation", m_autoBuilder, m_swerveDrive, m_fieldSim));
    SmartDashboard.putData("Auto Selector", m_autoChooser);

    m_autoChooser.addOption(
        "BalanceMid", new BalanceMid("BalanceMid", m_autoBuilder, m_swerveDrive, m_fieldSim));
    SmartDashboard.putData("Auto Selector", m_autoChooser);
    
    //m_autoChooser.addOption(
     // "JustBalance", new JustBalance("JustBalance", m_autoBuilder, m_swerveDrive, m_fieldSim));
  //SmartDashboard.putData("Auto Selector", m_autoChooser);
    
    


    //The width and length for the robot is done in meters on PathPlanner.

  }

  public void periodic() {
    m_fieldSim.periodic();
  }

  private void initAutoBuilder() {
    m_eventMap.put("wait", new WaitCommand(2));
  
    m_autoBuilder =
        new SwerveAutoBuilder(
            m_swerveDrive::getPoseMeters,
            m_swerveDrive::setOdometry,
            Constants.SwerveDrive.kSwerveKinematics,
            new PIDConstants(
                Constants.SwerveDrive.kP_Translation,
                Constants.SwerveDrive.kI_Translation,
                Constants.SwerveDrive.kD_Translation),
            new PIDConstants(
                Constants.SwerveDrive.kP_Rotation,
                Constants.SwerveDrive.kI_Rotation,
                Constants.SwerveDrive.kD_Rotation),
            m_swerveDrive::setSwerveModuleStatesAuto,
            m_eventMap,
            false,
            m_swerveDrive);
  }
  public void disableInit() {
    m_swerveDrive.setNeutralMode(IdleMode.kBrake);
  }

  public void teleopInit() {
    m_swerveDrive.setNeutralMode(IdleMode.kBrake);
    m_swerveDrive.resetState();
  }


  public void disabledInit() {}

  public void disabledPeriodic() {}

  public void autonomousInit() {}

  public void autonomousPeriodic() {}

  public void teleopPeriodic() {}

  public void simulationInit() {}

  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
