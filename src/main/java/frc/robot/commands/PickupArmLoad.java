/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PickupArmSubsystem;

public class PickupArmLoad extends CommandBase {
  /**
   * Creates a new PickupArmDown.
   */
  private final PickupArmSubsystem m_pickupArm;
  public PickupArmLoad(PickupArmSubsystem system) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pickupArm = system;
    addRequirements(m_pickupArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pickupArm.pickupArmLoad();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
