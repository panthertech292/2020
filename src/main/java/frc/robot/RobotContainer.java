/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final static XboxController m_driverController = new XboxController(Constants.driverController);
  private final static XboxController m_operController = new XboxController(Constants.operController);

  private final AnalogInput robotID = new AnalogInput(Constants.krobotID);
  //private double[] encoderTarget; 

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final PickupSubsystem m_pickupSubsystem = new PickupSubsystem();
  private final PickupArmSubsystem m_pickupArmSubsystem = new PickupArmSubsystem();
  private final TableSubsystem m_tableSubsystem = new TableSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final GateSubsystem m_gateSubsystem = new GateSubsystem();
  private final LiftSubsystem m_liftSubsystem = new LiftSubsystem();

  private final Command m_autoDead = new AutoDead(m_driveSubsystem);
  private final Command m_autoDriveForwardsTimed = new AutoCommandDriveForwardsTimed(m_driveSubsystem);
  private final Command m_autoDriveBackwardsTimed = new AutoCommandDriveBackwardsTimed(m_driveSubsystem);
  //private final Command m_autoDriveForwardsEncoder = new AutoCommandDriveForwardsEncoder(m_driveSubsystem);
  //private final Command m_autoDriveBackwardsEncoder = new AutoCommandDriveBackwardsEncoder(m_driveSubsystem);
  private final Command m_autoShootThenBackUp = new AutoShootThenBackUp(m_driveSubsystem, m_shooterSubsystem, m_gateSubsystem);



  private final Command m_autoTurn45DegreesLeft = new AutoTurn45DegreesLeft(m_driveSubsystem);
  private final Command m_autoTurn90DegreesLeft = new AutoTurn90DegreesLeft(m_driveSubsystem);
  //private final Command m_autoTurn180DegreesLeft = new AutoTurn180DegreesLeft(m_driveSubsystem);
  //private final Command m_autoTurn360DegreesLeft = new AutoTurn360DegreesLeft(m_driveSubsystem);
  private final Command m_autoTurn45DegreesRight = new AutoTurn45DegreesRight(m_driveSubsystem);
  private final Command m_autoTurn90DegreesRight = new AutoTurn90DegreesRight(m_driveSubsystem);
  private final Command m_autoTurn180DegreesRight = new AutoTurn180DegreesRight(m_driveSubsystem);
  //private final Command m_autoTurn360DegreesRight = new AutoTurn360DegreesRight(m_driveSubsystem);
  private final Command m_driveJogLeft = new DriveJogLeft(m_driveSubsystem);
  private final Command m_driveJogRight = new DriveJogRight(m_driveSubsystem);
 // private final Command m_liftStop = new LiftStop(m_liftSubsystem);
  private final Command m_liftRun = new LiftRun(m_liftSubsystem);
  
  private final Command m_driveTeleop = new DriveTeleop(m_driveSubsystem);
  private final Command m_pickupOff = new PickupOff(m_pickupSubsystem);
  //private final Command m_pickupOn = new PickupOn(m_pickupSubsystem);
  private final Command m_pickupAll = new PickupAll(m_pickupSubsystem, m_pickupArmSubsystem);
  private final Command m_pickupEncoderAll = new PickupEncoderAll(m_pickupSubsystem, m_pickupArmSubsystem);
  //private final Command m_pickupArmDown = new PickupArmDown(m_pickupArmSubsystem);
  private final Command m_pickupArmUp = new PickupArmUp(m_pickupArmSubsystem);
  private final Command m_pickupArmLoad = new PickupArmLoad(m_pickupArmSubsystem);
  //private final Command m_pickupArmDown = new PickupArmDown(m_pickupArmSubsystem);
  private final Command m_pickupArmEncoderUp = new PickupArmEncoderUp(m_pickupArmSubsystem);
  private final Command m_tableDown = new TableDown(m_tableSubsystem);
  private final Command m_tableUp = new TableUp(m_tableSubsystem);
  private final Command m_tableMiddle = new TableMiddle(m_tableSubsystem);
  //private final Command m_tableStop = new TableStop(m_tableSubsystem);
  private final Command m_shooterOff = new ShooterOff(m_shooterSubsystem);
  //private final Command m_shooterOn = new ShooterOn(m_shooterSubsystem);
  //private final Command m_shooterOnSmart = new ShooterOnSmart(m_shooterSubsystem);
  private final Command m_shootFromLine = new ShootFromLine(m_gateSubsystem, m_shooterSubsystem);
  private final Command m_shootFromTrench = new ShootFromTrench(m_gateSubsystem, m_shooterSubsystem);
  private final Command m_shootFromRendezvous = new ShootFromRendezvous(m_gateSubsystem, m_shooterSubsystem);
  //private final Command m_shooterFireGated = new ShooterFireGated(m_gateSubsystem, m_shooterSubsystem);
  //private final Command m_shooterFireGatedSmart = new ShooterFireGatedSmart(m_gateSubsystem, m_shooterSubsystem);
 // private final Command m_shooterFireGatedTrigger = new ShooterFireGatedTrigger(m_gateSubsystem, m_shooterSubsystem);
  //private final Command m_shooterFireGatedBallSensor = new ShooterFireGatedBallSensor(m_gateSubsystem, m_shooterSubsystem);
  //private final Command m_shooterFireGatedBallShooterSensor = new ShooterFireGatedBallShooterSensor(m_gateSubsystem, m_shooterSubsystem);
  //private final Command m_shooterFireGatedSingleShot = new ShooterFireGated(m_gateSubsystem, m_shooterSubsystem);
  //private final Command m_gate1Up = new Gate1Up(m_gateSubsystem);
  private final Command m_gate1Down = new Gate1Down(m_gateSubsystem);
  //private final Command m_printGateButton = new PrintGateButton(m_gateSubsystem);

  private double m_robotID;

  final static int m_leftTrigger = 2;
  final static int m_rightTrigger = 3;
  static int m_shooterMode = 0;


  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    m_robotID = getRobotID();
    System.out.println("RobotID = " + m_robotID);

    configureButtonBindings();


    m_driveSubsystem.setDefaultCommand(m_driveTeleop);
    m_pickupSubsystem.setDefaultCommand(m_pickupOff);
    m_tableSubsystem.setDefaultCommand(m_tableUp);
    m_shooterSubsystem.setDefaultCommand(m_shooterOff);
    m_gateSubsystem.setDefaultCommand(m_gate1Down);
    m_liftSubsystem.setDefaultCommand(m_liftRun);


    if(getRobotID() < 0.1) {
      m_pickupArmSubsystem.setDefaultCommand(m_pickupArmUp);
    } else {
      m_pickupArmSubsystem.setDefaultCommand(m_pickupArmEncoderUp);
      System.out.println("Backup Bot - default Command Pickup Arm Encoder");
    }


    m_chooser.addOption("Auto Dead", m_autoDead);
    m_chooser.addOption("Auto Forward Timed", m_autoDriveForwardsTimed);
    m_chooser.addOption("Auto Backward Timed", m_autoDriveBackwardsTimed);
    //m_chooser.addOption("Auto Forward Encoder", m_autoDriveForwardsEncoder);
    //m_chooser.addOption("Auto Backward Encoder", m_autoDriveBackwardsEncoder);
    m_chooser.addOption("Auto Turn 45 Degrees Left", m_autoTurn45DegreesLeft);
    m_chooser.addOption("Auto Turn 90 Degrees Left", m_autoTurn90DegreesLeft);
    //m_chooser.addOption("Auto Turn 180 Degrees Left", m_autoTurn180DegreesLeft);
    m_chooser.addOption("Auto Turn 45 Degrees Right", m_autoTurn45DegreesRight);
    m_chooser.addOption("Auto Turn 90 Degrees Right", m_autoTurn90DegreesRight);
    m_chooser.addOption("Auto Turn 180 Degrees Right", m_autoTurn180DegreesRight);
    //m_chooser.addOption("Auto Turn 360 Degrees Right", m_autoTurn360DegreesRight);
    //m_chooser.addOption("Auto Turn 360 Degrees Left", m_autoTurn360DegreesLeft);
    //m_chooser.addOption("Auto Shoot and Back", m_autoShootandBack);
    m_chooser.addOption("Auto Shoot and Back", m_autoShootThenBackUp);

    Shuffleboard.getTab("Autonomous").add(m_chooser);




   // encoderTarget[4, 11]

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    final JoystickButton o_aButton = new JoystickButton(m_operController, Button.kA.value);
    final JoystickButton o_bButton = new JoystickButton(m_operController, Button.kB.value);
    final JoystickButton o_xButton = new JoystickButton(m_operController, Button.kX.value);
    final JoystickButton o_yButton = new JoystickButton(m_operController, Button.kY.value);
    final JoystickButton o_start = new JoystickButton(m_operController, Button.kStart.value);
    final JoystickButton o_back = new JoystickButton(m_operController, Button.kBack.value);
    final JoystickButton o_lbumper = new JoystickButton(m_operController, Button.kBumperLeft.value);
    final JoystickButton o_rbumper = new JoystickButton(m_operController, Button.kBumperRight.value);
    //final POVButton o_POV0 = new POVButton(m_operController, 0);
    //final POVButton o_POV90 = new POVButton(m_operController, 90);
    //final POVButton o_POV180 = new POVButton(m_operController, 180);
    //final POVButton o_POV270 = new POVButton(m_operController, 270);

    final JoystickButton d_aButton = new JoystickButton(m_driverController, Button.kA.value);
    final JoystickButton d_bButton = new JoystickButton(m_driverController, Button.kB.value);
    //final JoystickButton d_xButton = new JoystickButton(m_driverController, Button.kX.value);
    //final JoystickButton d_yButton = new JoystickButton(m_driverController, Button.kY.value);
    //final JoystickButton d_start = new JoystickButton(m_driverController, Button.kStart.value);
    //final JoystickButton d_back = new JoystickButton(m_driverController, Button.kBack.value);
    //final JoystickButton d_lbumper = new JoystickButton(m_driverController, Button.kBumperLeft.value);
    //final JoystickButton d_rbumper = new JoystickButton(m_driverController, Button.kBumperRight.value);


    o_aButton.whenPressed(m_shootFromLine);
    o_bButton.whileHeld(m_tableMiddle);
    o_yButton.whileHeld(m_pickupAll);
    o_xButton.whileHeld(m_pickupArmLoad);
    o_start.whenPressed(m_tableDown);
    o_back.whenPressed(m_tableUp);
    o_lbumper.whenPressed(m_shootFromRendezvous);
    o_rbumper.whenPressed(m_shootFromTrench);

    if(getRobotID() < 0.1) {
      o_yButton.whileHeld(m_pickupAll);
    } else {
      o_yButton.whileHeld(m_pickupEncoderAll);
      System.out.println("Backup Bot - Y Command Pickup Encoder");
    }
/*
    if(getRobotID() < 0.1) {
      o_xButton.whileHeld(m_pickupArmUp);
    } else {
      o_xButton.whileHeld(m_pickupArmEncoderUp);
      System.out.println("Backup Bot - X Command Pickup Encoder Up");
    }
    */
    d_aButton.whenPressed(m_driveJogLeft);
    d_bButton.whenPressed(m_driveJogRight);
    //d_yButton.whileHeld(m_shooterOn);
    //d_xButton.whileHeld(m_shooterOnSmart);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public static double getLeftSpeed() {
    return m_driverController.getY(GenericHID.Hand.kLeft);
  }

  public static double getRightSpeed() {
    return m_driverController.getY(GenericHID.Hand.kRight);
    
  }
  public static double getLiftLeftSpeed() {
    double tempSpeed = m_operController.getY(GenericHID.Hand.kLeft);
   if( tempSpeed<.1){tempSpeed = 0;}
    return tempSpeed;
  }
  public static double getLiftRightSpeed() {
    double tempSpeed = m_operController.getY(GenericHID.Hand.kRight);
   if( tempSpeed<.1){tempSpeed = 0;}
    return tempSpeed;
  }

  public static double getShooterSpeedAdjust() {
    double m_shooterSpeedAdjust;
    m_shooterSpeedAdjust = (m_operController.getRawAxis(m_rightTrigger)/10) - (m_operController.getRawAxis(m_leftTrigger)/10);
    return m_shooterSpeedAdjust;
  }

  public static double getDriveSpeedAdjust() {
    double m_driveSpeedAdjust;
    m_driveSpeedAdjust =1-( (m_driverController.getRawAxis(m_rightTrigger)/2.5) + (m_driverController.getRawAxis(m_leftTrigger)/2.5));
    return m_driveSpeedAdjust;
  }
/*
  public static double getShooterSpeedAdjust() {
    double m_speedAdjust;
    int m_POV;
    m_POV = m_operController.getPOV();
    if
    return m_speedAdjust;
  }
*/

public static int getShooterMode() {
  return m_shooterMode;
}

public static void setShooterMode(int mode) {
  m_shooterMode = mode;
}
  

  public static boolean isFireGatedPressed(){
    return m_operController.getAButton() || m_operController.getBumper(Hand.kLeft) || m_operController.getBumper(Hand.kRight);
  }

  public static boolean isFireGatedSmartPressed(){
    return m_operController.getBackButton();
  }

public double getRobotID() {
    return robotID.getVoltage();
  }

  


  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
