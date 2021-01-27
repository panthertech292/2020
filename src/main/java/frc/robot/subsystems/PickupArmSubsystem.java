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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.PickupConstants;

public class PickupArmSubsystem extends SubsystemBase {
  /**
   * Creates a new PickupArmSubsystem.
   */
  private final WPI_TalonSRX pickupArm;
  private DigitalInput upLimitSwitch;
  private AnalogInput armPosition;

  private double m_armSpeed;

  private Encoder armEncoder;
  private double m_potCurrentValue;
  private double m_encoderCurrentValue;
  double m_setPointArm;

  

  private boolean m_limitSwitchTrue;


  public PickupArmSubsystem() {
    pickupArm = new WPI_TalonSRX(PickupConstants.kPickupArm);
    armEncoder = new Encoder(PickupConstants.pickupArmEncoder1, PickupConstants.pickupArmEncoder2);
    upLimitSwitch = new DigitalInput(PickupConstants.upLimitSwitch);
    armPosition = new AnalogInput(PickupConstants.kArmPosition);
    
    
    addChild("PickupArm", pickupArm);
    addChild("PickupArmEncoder", armEncoder);
    addChild("PickupArmLimit", upLimitSwitch);
    addChild("PickupArmPosition", armPosition);

    pickupArm.setNeutralMode(NeutralMode.Brake);
  }

  public void pickupArmStop() {
    m_armSpeed = PickupConstants.kPickupArmStop;
  }

  public void pickupArmUp() {
    // Note: The motor goes negative driving up and positive driving down
    // Note: The encoder goes negative driving down and positive going up (full up is zero)
    m_setPointArm = PickupConstants.upPositionPot; 
    m_potCurrentValue = getPotPosition();
    m_limitSwitchTrue = getUpLimitSwitch();
    if( m_limitSwitchTrue == true) {
      // If we are on the limit switch - do nothing
      m_armSpeed = PickupConstants.kPickupArmStop;
    } else {
      // If we are at the encoder setpoint, but not on the limit switch - drive slow to hit the switch
      if(m_potCurrentValue <= m_setPointArm){
        m_armSpeed = PickupConstants.kPickupArmHoldUp;
      } else {
        // If we are in main travel - go fast
        m_armSpeed = PickupConstants.kPickupArmTravelUp;
      }
    }
  }

    public void pickupArmUpEncoder() {
      // Note: The motor goes negative driving up and positive driving down
      // Note: The encoder goes negative driving down and positive going up (full up is zero)
      m_setPointArm = PickupConstants.upPosition; // Note the encoder runs backward (-500 -> 0)
      m_encoderCurrentValue = getEncoderPosition();
      m_limitSwitchTrue = getUpLimitSwitch();
      if( m_limitSwitchTrue == true) {
        // If we are on the limity switch - do nothing
        m_armSpeed = PickupConstants.kPickupArmStop;
        resetEncoderPosition();
      } else {
        // If we are at the encoder setpoint, but not on the limit switch - drive slow to hit the switch
        if(m_encoderCurrentValue >= m_setPointArm){
          m_armSpeed = PickupConstants.kPickupArmHoldUp;
        } else {
          // If we are in main travel - go fast
          m_armSpeed = PickupConstants.kPickupArmTravelUp;
        }
      }


  }

 
  public void pickupArmDown() {
    // Note: The motor goes negative driving up and positive driving down
    // Note: The encoder goes negative driving down and positive going up (full up is zero)
    m_setPointArm = PickupConstants.downPositionPot; // Note the encoder runs backward (-500 -> 0)
    m_potCurrentValue = getPotPosition();
    // If we are at the encoder setpoint, but not on the limit switch - drive slow to hit the switch
    if(m_potCurrentValue >= m_setPointArm){
      m_armSpeed = PickupConstants.kPickupArmHoldDown;
    } else {
        // If we are in main travel - go fast
      m_armSpeed = PickupConstants.kPickupArmTravelDown;
    }
  }

  public void pickupArmLoad(){
    m_setPointArm = PickupConstants.midPositionPot;
    m_potCurrentValue = getPotPosition();
    // If we are at the encoder setpoint, but not on the limit switch - drive slow to hit the switch
    if(m_potCurrentValue >= m_setPointArm){
      m_armSpeed = PickupConstants.kPickupArmStop-.075;
    } else {
        // If we are in main travel - go fast
      m_armSpeed = PickupConstants.kPickupArmTravelDown;
    }
  
  }

    public void pickupArmDownEncoder() {
      // Note: The motor goes negative driving up and positive driving down
      // Note: The encoder goes negative driving down and positive going up (full up is zero)
      m_setPointArm = PickupConstants.downPosition; // Note the encoder runs backward (-500 -> 0)
      m_encoderCurrentValue = getEncoderPosition();
        // If we are at the encoder setpoint, but not on the limit switch - drive slow to hit the switch
        if(m_encoderCurrentValue <= m_setPointArm){
          m_armSpeed = PickupConstants.kPickupArmHoldDown;
        } else {
          // If we are in main travel - go fast
          m_armSpeed = PickupConstants.kPickupArmTravelDown;
        }
      }


  public double getEncoderPosition(){
    return armEncoder.get();
  }

  public void resetEncoderPosition(){
    armEncoder.reset();
  }

  public double getEncoderDownSetpoint() {
    return  SmartDashboard.getNumber("Setpoint - Pichup Arm Encoder", 1);
  }

  public void changeSetPoints(double setPointArm){
    m_setPointArm = setPointArm;
  }
  public boolean getUpLimitSwitch() {
     return !upLimitSwitch.get();
  }

  public double getPotPosition() {
    return armPosition.getVoltage();
  }

  public double getArmSetPoint() {
    return m_setPointArm;
 }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pickupArm.set(-m_armSpeed);
    //System.out.println("PickupArm Speed = " + m_armSpeed);
    SmartDashboard.putNumber("Pickup Arm Speed", m_armSpeed);
    SmartDashboard.putNumber("Pickup Arm Encoder", getEncoderPosition());
    SmartDashboard.putNumber("Pickup Arm SetPoint", getArmSetPoint());
    SmartDashboard.putBoolean("Pickup Arm LimitSW", getUpLimitSwitch());
    SmartDashboard.putNumber("Pickup Arm Pot Position", getPotPosition());
  }
}
