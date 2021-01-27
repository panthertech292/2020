/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveEncoder extends CommandBase {
  /**
   * Creates a new AutoDrive.
   */
  private final DriveSubsystem m_drive;
  private double m_leftspeed;
  private double m_rightspeed;
  //private double m_time;
  private double m_leftDistance;
  //private double m_rightDistance;
  

  //private final Timer timer;
  public AutoDriveEncoder(double leftspeed, double rightspeed, double time, double leftdistance, double rightdistance, DriveSubsystem subsystem){
    m_drive = subsystem;
    m_leftspeed = leftspeed;
    m_rightspeed = rightspeed;
    //m_time = time;
    m_leftDistance = leftdistance;
    //m_rightDistance = rightdistance;
   


    addRequirements(subsystem);
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    //m_drive.resetTimer();
    m_drive.zeroLeftPosition();
    m_drive.zeroRightPosition();
    
    m_drive.DriveModeSetPoint();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //System.out.println("left = " + m_leftspeed + " right = " + m_rightspeed);
    //if (m_drive.getLeftPosition()<m_leftDistance||m_rightDistance>m_drive.getRightPosition()){
  m_drive.changeSetPoints(m_leftspeed, m_rightspeed);

}
  

    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // m_drive.differentialDrive(0, 0);
   m_drive.changeSetPoints(0.0, 0.0);
   System.out.println("Encoders done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     //boolean tempDone;

    //return m_drive.getTimerValue()>m_time;

    //tempDone =  m_leftDistance < (Math.abs(m_drive.getLeftPosition() - m_drive.getZeroPosition()));
    //System.out.println("ADE isFinished = " + "actual = " + m_drive.getLeftPosition() + " zero = " + m_drive.getZeroLeft());
    //System.out.println("ADE isFinished = " + tempDone);
    return m_drive.encoderFinish(m_leftDistance);
    //return m_leftDistance < (Math.abs(m_drive.getLeftPosition() - m_drive.getZeroLeft()));
    //|| (m_rightDistance < (Math.abs(m_drive.getRightPosition() - m_drive.getZeroRight())));
  }
}
