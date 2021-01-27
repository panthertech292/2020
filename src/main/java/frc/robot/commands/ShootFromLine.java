/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GateSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShooterFireGatedBallSensorRange;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootFromLine extends SequentialCommandGroup {
  /**
   * Creates a new ShootFromLine.
   */
  private final GateSubsystem m_gate;
  private final ShooterSubsystem m_shooter;
  //private double m_range = 0;
  public ShootFromLine(GateSubsystem subsystem1, ShooterSubsystem subsystem2) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    //super();
    m_gate = subsystem1;
    m_shooter = subsystem2;
    addCommands(
      new ShooterFireGatedBallSensorRange(m_gate, m_shooter,ShooterConstants.kShooterLine)


    );
  }
}
