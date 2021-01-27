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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.TableConstants;

public class TableSubsystem extends SubsystemBase {
  /**
   * Creates a new PickupArmSubsystem.
   */
  private final WPI_TalonSRX tableMotor;
  private double m_tableSpeed = 0.0;

  private Encoder tableEncoder;
  private double m_encoderCurrentValue;
  private double m_setPointTable;

  private DigitalInput upLimitSwitch;
  private boolean m_limitSwitchTrue;


  public TableSubsystem() {
    tableMotor = new WPI_TalonSRX(TableConstants.kTableMotor);
    tableEncoder = new Encoder(TableConstants.ktableEncoderChannel1, TableConstants.ktableEncoderChannel2);
    upLimitSwitch = new DigitalInput(TableConstants.upLimitSwitch);
    
    m_tableSpeed = 0.0;

    addChild("TableMotor", tableMotor);
    addChild("TableEncoder", tableEncoder);
    addChild("TableLimit", upLimitSwitch);

    tableMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void TableStop() {
    m_tableSpeed = TableConstants.kTableStop;
  }

  public void TableUp() {
    // Note: The motor goes negative driving up and positive driving down
    // Note: The encoder goes negative driving down and positive going up (full up is zero)
    m_setPointTable = TableConstants.upPosition; // Note the encoder runs backward (-500 -> 0)
    m_encoderCurrentValue = getEncoderPosition();
    m_limitSwitchTrue = getUpLimitSwitch();
    if( m_limitSwitchTrue == true) {
      // If we are on the limity switch - do nothing
      m_tableSpeed = TableConstants.kTableStop;
      resetEncoderPosition();
    } else {
      // If we are at the encoder setpoint, but not on the limit switch - drive slow to hit the switch
      if(m_encoderCurrentValue >= m_setPointTable){
        m_tableSpeed = TableConstants.kTableHoldUp;
      } else {
        // If we are in main travel - go fast
        m_tableSpeed = TableConstants.kTableTravelUp;
      }
    }




  }

 
  public void TableDown() {
    // Note: The motor goes negative driving up and positive driving down
    // Note: The encoder goes negative driving down and positive going up (full up is zero)
    m_setPointTable = TableConstants.downPosition; // Note the encoder runs backward (-500 -> 0)
    m_encoderCurrentValue = getEncoderPosition();
      // If we are at the encoder setpoint, but not on the limit switch - drive slow to hit the switch
      if(m_encoderCurrentValue <= m_setPointTable){
        m_tableSpeed = TableConstants.kTableStop;
      } else {
        // If we are in main travel - go fast
        m_tableSpeed = TableConstants.kTableTravelDown;
      }
    }

    public void TableMiddle(){
      m_setPointTable = TableConstants.middlePosition; // Note the encoder runs backward (-500 -> 0)
    m_encoderCurrentValue = getEncoderPosition();
    
    
      // If we are at the encoder setpoint, but not on the limit switch - drive slow to hit the switch
      if(m_encoderCurrentValue >= m_setPointTable){
        m_tableSpeed = (TableConstants.kTableHoldUp/2);
      } else {
        // If we are in main travel - go fast
        m_tableSpeed = TableConstants.kTableTravelUp;
      }
    }

    


  public double getEncoderPosition(){
    return tableEncoder.get();
  }

  public void resetEncoderPosition(){
    tableEncoder.reset();
  }
  public void changeSetPoints(double setPointTable){
    m_setPointTable = setPointTable;
  }

  public double getEncoderDownSetpoint() {
    return  SmartDashboard.getNumber("Setpoint - Table Encoder", 1);
  }

  public boolean getUpLimitSwitch() {
     return !upLimitSwitch.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tableMotor.set(m_tableSpeed);
    SmartDashboard.putNumber("Table Speed", m_tableSpeed);
    //SmartDashboard.putNumber("Pickup Arm Encoder", getEncoderPosition());
    //SmartDashboard.putNumber("Table SetPoint", getArmSetPoint());
    SmartDashboard.putBoolean("Table LimitSW", getUpLimitSwitch());
    SmartDashboard.putNumber("Table Position", getEncoderPosition());
  }
}

