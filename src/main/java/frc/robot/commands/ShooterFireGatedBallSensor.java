/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.GateSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterFireGatedBallSensor extends CommandBase {
  /**
   * Creates a new ShooterFireGatedSingleShot.
   */
  private final GateSubsystem m_gateSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private int m_shootState;
  private boolean m_done;
  private int m_shootCount;
 
  public ShooterFireGatedBallSensor(GateSubsystem gatesubsystem, ShooterSubsystem shootersubsystem) {
    m_gateSubsystem = gatesubsystem;
    m_shooterSubsystem = shootersubsystem;
    m_shootState = 1;
    m_done = false;
    m_shootCount = 0;
    addRequirements(m_gateSubsystem, m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shootState = 1;
    m_done = false;
    m_shootCount = 0;
    m_shooterSubsystem.changeSetSpeed(ShooterConstants.kShooterFull);
    m_gateSubsystem.resetGates();
   

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_shootState == 1){
      // Wait for shooter to get going
      if(m_gateSubsystem.isShooterMotorSpunUp()) {m_shootState = 2; }
    }
    //
    if(m_shootState == 2) {
      //Start the shooter cycle
      m_gateSubsystem.ShootOne();
      m_shootState = 3;
    }
    if(m_shootState == 3) {
      // Wait for shooter cycle to get done
      if(m_gateSubsystem.isShooterDone()) { 
        m_shootState = 4;
      }
    }
      //
    if(m_shootState == 4) {
      // Start the reload cycle
      m_gateSubsystem.LoadNext();
      m_shootCount = m_shootCount + 1;
      
      if(m_gateSubsystem.isFireGatedButtonStillPressed()) {
        m_shootState = 5;
      } else {
        m_done = true;
        //System.out.println("Done - No button");
      }
      if(m_shootCount >= 5) {
        m_done = true;
        //System.out.println("Done - Count = " + m_shootCount);
      }
    }
      //
    if(m_shootState == 5){
      // Wait for loader cycle to get done
      if(m_gateSubsystem.isLoaderDone()) {
        if(m_gateSubsystem.isBallPresent() == false){m_shootCount = 4;}
        m_shootState = 2;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gateSubsystem.resetShootStateEnd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_done;
  }
}
