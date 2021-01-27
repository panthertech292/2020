/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterOnSmart extends CommandBase {
  /**
   * Creates a new ShooterOn.
   */
  private final ShooterSubsystem m_shooter;

  public ShooterOnSmart(ShooterSubsystem system) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = system;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shooter On!");
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.changeSetSpeed(SmartDashboard.getNumber("Set the Shooter here - Back Button", ShooterConstants.kShooterNormal));
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
