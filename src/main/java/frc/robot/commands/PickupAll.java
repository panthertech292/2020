/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.PickupArmSubsystem;
import frc.robot.commands.PickupArmDown;
import frc.robot.commands.PickupOn;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PickupAll extends ParallelCommandGroup {
  /**
   * Creates a new PickupAll.
   */
  public PickupAll(PickupSubsystem pickup, PickupArmSubsystem pickupArm) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    addCommands(

      new PickupOn(pickup),
      new PickupArmDown(pickupArm)

    );
  }
}
