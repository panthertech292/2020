package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final WPI_TalonSRX frontLeftMotor;
  private final WPI_TalonSRX frontRightMotor;
  private final WPI_TalonSRX backLeftMotor;
  private final WPI_TalonSRX backRightMotor;

  private final SpeedControllerGroup m_leftside;
  private final SpeedControllerGroup m_rightside;
 
private final Timer timer;

    private final DifferentialDrive m_drive;

    private double m_leftspeed;
    private double m_rightspeed;
    private double m_setpointleft;
    private double m_setpointright;

    private double zeroLeftPosition;
    private double zeroRightPosition;
    //private double m_leftDistance;

    private double encoderConversion;
     
    private int m_modeTeleop = 0;
    private int m_modeSetPoint = 1;
    private int m_driveMode = m_modeTeleop;
 



  public DriveSubsystem() {

    frontLeftMotor = new WPI_TalonSRX(DriveConstants.kFrontLeftMotor);
    frontRightMotor = new WPI_TalonSRX(DriveConstants.kFrontRightMotor);
    backLeftMotor = new WPI_TalonSRX(DriveConstants.kBackLeftMotor);
    backRightMotor = new WPI_TalonSRX(DriveConstants.kBackRightMotor);
  
    m_leftside = new SpeedControllerGroup(frontLeftMotor,backLeftMotor);
    m_rightside = new SpeedControllerGroup(frontRightMotor, backRightMotor);
    //m_leftDistance = 0.0;

    m_drive = new DifferentialDrive(m_leftside,m_rightside);

  backLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  backRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

  m_setpointleft = 0.0;
  m_setpointright = 0.0;


    timer = new Timer();
    encoderConversion = 4096;


    addChild("Drive", m_drive);
    addChild("Left Side", m_leftside);
    addChild("Right Side", m_rightside);

    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
      
  }
  public void differentialDrive(double leftspeed, double rightspeed) {
    m_leftspeed = -leftspeed;
    m_rightspeed = - rightspeed;
    m_drive.tankDrive(m_leftspeed, m_rightspeed);
  }
    public void resetTimer() {
      timer.reset();
      timer.start();
    }
  
  public double getTimerValue() {
    return Math.abs(timer.get());
  }
  
  
  public double getLeftEncoderValue() {
      return backLeftMotor.getSelectedSensorPosition();
  }
  
  public double getLeftPosition(){
    return ((backLeftMotor.getSelectedSensorPosition() - zeroLeftPosition)/encoderConversion)*(Math.PI*6);
  }
  
  public void zeroLeftPosition() {
    zeroLeftPosition = backLeftMotor.getSelectedSensorPosition();
  }
  public double getZeroPosition(){
    return zeroLeftPosition;
  }
  public void zeroRightPosition(){
    zeroRightPosition = backRightMotor.getSelectedSensorPosition();
  }
  
  public double getRightPosition(){
    return ((backRightMotor.getSelectedSensorPosition() - zeroRightPosition)/encoderConversion)*(Math.PI*6);
  }
  
  public double getRightEncoderValue(){
      return backRightMotor.getSelectedSensorPosition();
  }
  
  public void changeSetPoints(double setPointLeft, double setPointright){
    m_setpointleft = setPointLeft;
    m_setpointright = setPointright;
  }
  
  public void DriveModeTeleop() {
    m_driveMode = m_modeTeleop;
  }

  public void DriveModeSetPoint() {
    m_driveMode = m_modeSetPoint;
  }

  public boolean encoderFinish( double distance){
    return distance < (Math.abs(getLeftPosition()));
  }
  
  public double rotateRobot(double m_angle){
    return ((21*Math.PI)*(m_angle/360));
  }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Drive Encoder Position ", getLeftPosition());
      if (m_driveMode == m_modeTeleop) {
        differentialDrive((RobotContainer.getLeftSpeed()*RobotContainer.getDriveSpeedAdjust()), (RobotContainer.getRightSpeed()*RobotContainer.getDriveSpeedAdjust()));
      } else {
      differentialDrive(-m_setpointleft, -m_setpointright);
      //System.out.println(m_setpointleft);
     // System.out.println(m_setpointright);
      // This method will be called once per scheduler run
      }
    }
  }