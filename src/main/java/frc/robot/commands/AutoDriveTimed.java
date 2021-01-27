/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveTimed extends CommandBase {
  /**
   * Creates a new AutoDrive.
   */
  private final DriveSubsystem m_drive;
  private double m_leftspeed;
  private double m_rightspeed;
  private double m_time;

  public AutoDriveTimed(double leftspeed, double rightspeed, double time, DriveSubsystem subsystem) {
    m_drive = subsystem;
    m_leftspeed = leftspeed;
    m_rightspeed = rightspeed;
    m_time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    m_drive.resetTimer();
    m_drive.DriveModeSetPoint();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  m_drive.changeSetPoints(m_leftspeed, m_rightspeed);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.changeSetPoints(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     
    return m_drive.getTimerValue()>m_time;
  }
}
