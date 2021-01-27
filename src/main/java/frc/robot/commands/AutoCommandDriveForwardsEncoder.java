/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoCommandDriveForwardsEncoder extends SequentialCommandGroup {
  /**
   * Creates a new AutoCommandDriveForwardEncoder.
   */
  private final DriveSubsystem m_driveSubsystem;
  public AutoCommandDriveForwardsEncoder(DriveSubsystem subsystem1) {
    m_driveSubsystem = subsystem1;
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addRequirements(m_driveSubsystem);

    //System.out.println("Command = ACDFE <<<<<<<<");
    addCommands(
      

    new AutoDriveEncoder(DriveConstants.autoForwardSpeed, DriveConstants.autoForwardSpeed, 10.0, 100.0, 100.0, m_driveSubsystem)


    ); 
  }
}
