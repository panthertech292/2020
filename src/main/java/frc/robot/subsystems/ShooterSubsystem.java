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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final WPI_TalonSRX shooterMotor;
  private double m_shooterSpeed = 0;
  private final Timer timer;
  private Encoder shooterEncoder;
  private double m_EncoderSetPointShooter;
 /* private int counter;
  //private double e1;
  private double e2;
  private double e3;
  private double e4;
  private boolean close;
  private double threshold;
  private double previousEncoderRate; */
  private PowerDistributionPanel PDP;

  public ShooterSubsystem() {
    shooterMotor = new WPI_TalonSRX(ShooterConstants.kShooterMotor);
    shooterEncoder = new Encoder(ShooterConstants.kshooterEncoderChannel1, ShooterConstants.kshooterEncoderChannel2);
    timer = new Timer();
    PDP = new PowerDistributionPanel();

    m_shooterSpeed = 0.0;

    addChild("ShooterMotor", shooterMotor);
    addChild("ShooterEncoder", shooterEncoder);
    shooterMotor.setNeutralMode(NeutralMode.Brake);

  }

  public void ShooterStop() {
    m_shooterSpeed = ShooterConstants.kShooterStop;
  }

  public void resetTimer() {
    timer.reset();
    timer.start();
  }

  public Double getTimerValue() {
    return Math.abs(timer.get());
  }

  public double getEncoderPosition(){
    return shooterEncoder.get();
  }

  public double getEncoderRate(){
    return shooterEncoder.getRate();
  }

  public void resetEncoderPosition(){
    shooterEncoder.reset();
  }

  public void changeSetSpeed(double desiredSpeed){
    m_shooterSpeed = desiredSpeed;
    //System.out.println("Shooter ChangeSetSpeed ########################");
  }

  public void changeEncoderSetPoints(double desiredEncoder){
    m_EncoderSetPointShooter = desiredEncoder;
  }

 /* public void changeRange(double desiredRange){
    m_shooterSpeed = ShooterConstants.kSpeedScale * desiredRange + ShooterConstants.kSpeedConstant;
    m_EncoderSetPointShooter = ShooterConstants.kEncoderScale * desiredRange + ShooterConstants.kEnocderConstant;
  }

  public void getShooterMode(double desiredRange){
    m_shooterSpeed = ShooterConstants.kSpeedScale * desiredRange + ShooterConstants.kSpeedConstant;
    m_EncoderSetPointShooter = ShooterConstants.kEncoderScale * desiredRange + ShooterConstants.kEnocderConstant;
  } */


  public double getEncoderDownSetpoint() {
    return  SmartDashboard.getNumber("Setpoint - Shooter Encoder", 1);
  }
/*
  public double QuickPid(double set, double actual) {
      double m_set;
      double m_actual;
      double delta = 0; double previousdelta; double integral = 0; double derivative = 0.0;
      double p = 1.0; double i = 0.0; double d = 0.0; double rcw = 0.0;
      double timeInterval = 0.02;
      
      m_set = set;
      m_actual = actual;
      
      delta = m_set - m_actual;
      integral += delta * timeInterval;
      derivative = (delta - previousdelta) / 0.02;



      previousdelta = delta;

  }
*/
public int checkPDPVoltage(){
  return (int) Math.floor(PDP.getVoltage()*10);
}

/*public double getDesiredEncoderSetpoint(){
  int voltage = (checkPDPVoltage()-100);
 int power = (int) ((Math.floor(m_shooterSpeed*10))-1.0);

 int index = ((power*100)+voltage);



 

}*/


public double getEstimatedEncoderValue(){
double encoderValue = 0;
double m_shooterFloor = Math.floor(m_shooterSpeed*10);
if(m_shooterFloor <= 4){
  encoderValue = 150000;
}
if(m_shooterFloor == 5){
  encoderValue = 185000;
}
if(m_shooterFloor == 6){
  encoderValue = 220000;
}
if(m_shooterFloor == 7){
  encoderValue = 255000;
}

if(m_shooterFloor == 8){
  encoderValue = 290000;
}

if(m_shooterFloor == 9){
  encoderValue = 325000;
}
if(m_shooterFloor == 10){
  encoderValue = 360000;
}


return encoderValue;
}



/*public void initCloseEnough(){
  double set = getEncoderRate();
  e1 = set;
  e2 = set;
  e3 = set;
  e4 = set;
  counter = 0;
  
  previousEncoderRate = set;
  threshold = ShooterConstants.kthreshold;
}

public boolean isCloseEnough(){
  
double actual;
double delta;
double aveDelta;
//System.out.println(counter);
counter= counter+1;
if(counter >= 4 && getEncoderRate() >= 250000){
   counter = 0;
  actual = getEncoderRate();
  
  delta = previousEncoderRate - actual;
  aveDelta = (delta + e2 + e3 + e4)/4;
  //System.out.println("aveDelta="+ aveDelta+ "Actual ="+ actual);



  if(aveDelta < threshold){
  close = true;

  } else{
    close = false;
  }
  e1 = e2;
  e2 = e3;
  e3 = e4;
  e4 = delta;
  return close;
}else {
  return close;
}
}
*/


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterMotor.set(m_shooterSpeed+RobotContainer.getShooterSpeedAdjust());
    SmartDashboard.putNumber("ShooterSpeed", m_shooterSpeed);
    SmartDashboard.putNumber("ShooterEncoderRateSet", m_EncoderSetPointShooter);
    SmartDashboard.putNumber("ShooterEncoderRate", getEncoderRate());
  }
}

