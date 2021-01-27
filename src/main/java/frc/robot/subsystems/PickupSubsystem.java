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
import frc.robot.Constants.PickupConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PickupSubsystem extends SubsystemBase {
  /**
   * Creates a new PickupSubsystem.
   */
  private final WPI_TalonSRX pickupMotorR;
  private final WPI_TalonSRX pickupMotorL;
  private double m_pickupSpeed1 = 0.0;
  private double m_pickupSpeed2 = 0.0;
  private double m_ratio = 1.3;

  public PickupSubsystem() {
    pickupMotorR = new WPI_TalonSRX(PickupConstants.kPickupMotorR);
    pickupMotorL = new WPI_TalonSRX(PickupConstants.kPickupMotorL);
    m_ratio = PickupConstants.kRollerRatio;

    addChild("PickupR", pickupMotorR);
    addChild("PickupL", pickupMotorL);
    pickupMotorL.setNeutralMode(NeutralMode.Brake);
    pickupMotorR.setNeutralMode(NeutralMode.Brake);
  }

  public void ChangeSetpoint(double pickupspeed) {
    m_pickupSpeed1 = pickupspeed; //* m_ratio;
    m_pickupSpeed2 = pickupspeed;

  }

  public void ChangeSetpointsRev(double pickupspeed) {
    m_pickupSpeed1 = -pickupspeed; // * m_ratio;
    m_pickupSpeed2 = -pickupspeed;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      pickupMotorR.set(.5);
      pickupMotorL.set(.5);
      SmartDashboard.putNumber("Pickup Speed 1", m_pickupSpeed1);
      SmartDashboard.putNumber("Pickup Speed 2", m_pickupSpeed2);
      System.out.println(m_pickupSpeed1);
      System.out.println(-m_pickupSpeed2);
    
  }
}



