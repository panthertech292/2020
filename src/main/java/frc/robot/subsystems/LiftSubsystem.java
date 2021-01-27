/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.LiftConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class LiftSubsystem extends SubsystemBase {
  /**
   * Creates a new PickupArmSubsystem.
   */
  private final WPI_TalonSRX liftMotorLeftUp; 
  private final WPI_TalonSRX liftMotorLeftDown; 
  private final WPI_TalonSRX liftMotorRightUp; 
  private final WPI_TalonSRX liftMotorRightDown; 
  private final SpeedControllerGroup liftLeftGroup;
  private final SpeedControllerGroup liftRightGroup;
  private DigitalInput liftLimitSwitch;
  private double m_liftSpeedLeft = 0.0;
  private double m_liftSpeedRight = 0.0;

  private AnalogInput kArmLimitSwitchLeft;
  private AnalogInput kArmLimitSwitchRight;


  public LiftSubsystem() {
    liftMotorLeftUp = new WPI_TalonSRX(LiftConstants.kLiftMotorLeftUp);
    liftMotorLeftDown = new WPI_TalonSRX(LiftConstants.kLiftMotorLeftDown);
    liftMotorRightUp = new WPI_TalonSRX(LiftConstants.kLiftMotorRightUp);
    liftMotorRightDown = new WPI_TalonSRX(LiftConstants.kLiftMotorRightDown);
    liftLimitSwitch = new DigitalInput(LiftConstants.liftLimitSwitch);
    liftLeftGroup = new SpeedControllerGroup(liftMotorLeftUp, liftMotorLeftDown);
    liftRightGroup = new SpeedControllerGroup( liftMotorRightDown, liftMotorRightUp);

    kArmLimitSwitchLeft = new AnalogInput(LiftConstants.kArmLimitSwitchLeft);
    kArmLimitSwitchRight = new AnalogInput(LiftConstants.kArmLimitSwitchRight);

    m_liftSpeedLeft = 0.0;
    m_liftSpeedRight = 0.0;
    addChild("LiftMotor", liftMotorLeftUp);
    addChild("LiftLimit", liftLimitSwitch);

    liftMotorLeftDown.setNeutralMode(NeutralMode.Brake);
    liftMotorLeftUp.setNeutralMode(NeutralMode.Brake);
    liftMotorRightDown.setNeutralMode(NeutralMode.Brake);
    liftMotorRightUp.setNeutralMode(NeutralMode.Brake);
  }

  public void changeSetPointsLeft(double setPointLift){
    m_liftSpeedLeft = setPointLift;
  }
  public void changeSetPointsRight(double setPointLift){
    m_liftSpeedRight = setPointLift;
  }


  public void liftRunLeft(){
    final double m_tempSpeed = RobotContainer.getLiftLeftSpeed();
    if (readLeftLimitSwitch()==false){
        changeSetPointsLeft(m_tempSpeed);
    }
    else{liftStopLeft();}

  }
  public void liftRunRight(){
    final double m_tempSpeed = RobotContainer.getLiftRightSpeed();
    if (readRightLimitSwitch()==false){
        changeSetPointsRight(m_tempSpeed);
    }
    else{liftStopRight();}

  }
  public void liftStopLeft(){
    changeSetPointsLeft(LiftConstants.kLiftStop);}

  public void liftStopRight(){
    changeSetPointsRight(LiftConstants.kLiftStop);
    
  }

  public boolean getLiftLimitSwitch(){
    return liftLimitSwitch.get();
  }

  public boolean readLeftLimitSwitch(){
    boolean m_leftDone = false;
  if(kArmLimitSwitchLeft.getAverageVoltage()<2.5){m_leftDone = false;}
  else{m_leftDone = true;}
  return m_leftDone;
  
   }
   public boolean readRightLimitSwitch(){
    boolean m_rightDone = false;
  if(kArmLimitSwitchRight.getAverageVoltage()<2.5){m_rightDone = false;}
  else{m_rightDone = true;}
  return m_rightDone;}
   
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    liftLeftGroup.set(m_liftSpeedLeft);
    liftRightGroup.set(m_liftSpeedRight);
    
    SmartDashboard.putNumber("Lift Speed", m_liftSpeedLeft);
  }
}

