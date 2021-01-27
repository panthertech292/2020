/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GateSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootThenBackUp extends SequentialCommandGroup {
  /**
   * Creates a new AutoShootThenBackUp.
   */
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final GateSubsystem m_gateSubsystem;
  
  public AutoShootThenBackUp(DriveSubsystem subsystem1, ShooterSubsystem subsystem2, GateSubsystem subsystem3) {
    m_driveSubsystem = subsystem1;
    m_shooterSubsystem = subsystem2;
    m_gateSubsystem = subsystem3;
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    addCommands(
      new ShooterFireGated3(m_gateSubsystem, m_shooterSubsystem),
      new AutoCommandDriveBackwardsTimed(m_driveSubsystem)
    );
    
  }
}
