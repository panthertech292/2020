/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    static final int operController = 1; // USB
    static final int driverController = 0; // USB
    public static final int krobotID = 6; // AIO


    public static final int resolutionHeight = 480;
    public static final int resolutionWidth = 640;

    public static final int scaledHeight = 10;
    public static final int scaledWidth = 10;

    public static final double scaledCenterX = 4.7;
    public static final double scaledCenterY = 3.0;

    public static final double scaledRectHeight = 2.5;
    public static final double scaledRectWidth = ((3*scaledRectHeight)/4);

    
    public static final double rectCenterX = ((scaledCenterX/scaledWidth)*resolutionWidth);
    public static final double rectCenterY = ((scaledCenterY/scaledHeight)*resolutionHeight);
    public static final double rectWidth = ((scaledRectWidth/scaledWidth)*resolutionWidth);
    public static final double rectHeight = ((scaledRectHeight/scaledHeight)*resolutionHeight);

    
    

    public static final class DriveConstants {
        //
        // Ports
        //
        public static final int kFrontLeftMotor = 0; // CAN
        public static final int kBackLeftMotor = 1;  // CAN
        public static final int kFrontRightMotor = 2; // CAN
        public static final int kBackRightMotor = 3; // CAN
        //public static final int leftEncoderChannel1 = 4; // CAN
        //public static final int leftEncoderChannel2 = 5; // CAN
        //public static final int rightEncoderChannel1 = 6; // CAN
        //public static final int rightEncoderChannel2 = 7; // CAN
        //
        // Setpoints
        //
        public static final double leftDriveBackHalf = -0.5;
        public static final double rightDriveBackHalf = -0.5;
        public static final double leftDriveForwardHalf = 0.5;
        public static final double rightDriveForwardHalf = 0.5;
        public static final double autoForwardSpeed = 0.7;
        public static final double autoBackwardSpeed = -0.7;
        public static final double jogAngleLeft = 1;
        public static final double jogAngleRight = 1;
    }
    public static final class PickupConstants {
        //
        // Ports
        //
        public static final int kPickupMotorR = 10; // CAN Inverted
        public static final int kPickupMotorL = 6; // CAN Inverted
        public static final int kPickupArm = 25;  // CAN
        public static final int pickupArmEncoder1 = 4; // DIO
        public static final int pickupArmEncoder2 = 5; // DIO
        public static final int upLimitSwitch = 9;  // DIO
        public static final int kArmPosition = 0; // AIO

        
        //
        // Setpoints
        //
        public static final double upPosition = 0.0;
        public static final double downPosition = -430.0;
        public static final double upFullPositionPot = 2.00;
        public static final double upPositionPot = upFullPositionPot+.07;
        public static final double downFullPositionPot = 2.25; 
        public static final double downPositionPot = downFullPositionPot-.1; 
        public static final double midPositionPot = 2.09; 
        public static final double kPickupOff = 0.0;
        public static final double kPickupSpeed = 0.325;

        // How hard to drive the arm when full down
        public static final double kPickupArmHoldDown = 0.075;
        public static final double kPickupArmTravelDown = 0.7;
        public static final double kPickupArmStop = 0.0;
        // How hard to drive the arm when up, but not on limit sw
        public static final double kPickupArmHoldUp = -0.2;
        public static final double kPickupArmTravelUp = -0.8;
		public static final double kRollerRatio = 1.3;


    }

    public static final class ShooterConstants {
        // Ports
        public static final int kShooterMotor = 4; // CAN 
        public static final int kshooterEncoderChannel1 = 2; // DIO
        public static final int kshooterEncoderChannel2 = 3; // DIO
        // 
        public static final double kShooterStop = 0.0;
        public static final double kShooterMeh = 0.4;
        public static final double kShooterExactlyHalf = 0.5;
        public static final double kShooterNormal = 0.6;
        public static final double kShooterFull = 0.8;
        public static final double kShooterWow = 1.0;
        public static final double kShooterMax = 1.0;
        //
        public static final double kShooterAuto1 = 0.6;
        public static final double kShooterLine = 0.6;
        public static final double kShooterTrench = 0.8;
        public static final double kShooterRond = 1.0;
        //
        public static final double kShooterModeLine = 0;
        public static final double kShooterModeTrench = 1;
        public static final double kShooterModeRond = 2;
        //
		public static final double kSpeedScale = 0;
		public static final double kSpeedConstant = 0;
		public static final double kEncoderScale = 0;
        public static final double kEnocderConstant = 0;
        public static final double kthreshold = 10000;


       






    }
    public static final class TableConstants{
        // Ports
        public static final int kTableMotor = 8; // CAN
        public static final int ktableEncoderChannel1 = 0; //DIO
        public static final int ktableEncoderChannel2 = 1; // DIO
        public static final int upLimitSwitch = 8; // DIO?

        //
        public static final double upPosition = 0.0;
        public static final double middlePosition = 180.0;
        public static final double downPosition = -550.0;
        // How hard to drive the arm when full down
        public static final double kTableHoldDown = -0.1;
        public static final double kTableTravelDown = -0.5;
        public static final double kTableStop = 0.0;
        // How hard to drive the arm when up, but not on limit sw
        public static final double kTableHoldUp = 0.1;
        public static final double kTableTravelUp = 0.5;
    }
    public static final class LiftConstants{
        // Ports
        public static final int kLiftMotorLeftUp = 12;
        public static final int kLiftMotorLeftDown = 9;
        public static final int kLiftMotorRightUp = 11;
        public static final int kLiftMotorRightDown = 29; // CAN
        public static final int kLiftEncoderChannel1 = 0; //DIO
        public static final int kLiftEncoderChannel2 = 1; // DIO
        public static final int liftLimitSwitch = 6; // DIO?

        //
        public static final double upPosition = 0.0;
        public static final double downPosition = 500.0;
        // How hard to drive the arm when full down
        public static final double kLiftHoldDown = -0.1;
        public static final double kLiftTravelDown = -0.5;
        public static final double kLiftStop = 0.0;
        // How hard to drive the arm when up, but not on limit sw
        public static final double kLiftHoldUp = 0.1;
        public static final double kLiftTravelUp = 0.5;

        public static final int kArmLimitSwitchLeft = 2; //Analog Input
        public static final int kArmLimitSwitchRight = 3; //Analog Input
    }
    public static final class GateConstants{
        public static final int gate1Servo = 0; // PWM
        public static final int ball1sensor = 7; // DIO
        public static final double gate1Down = 1.0;
        public static final double gate1Half = 0.75;
        public static final double gate1Up = 0.5;

        //
        public static final double gateUpTime = 0.5;
        public static final double gateDownTime = 1.5;
		public static final double ShooterSpinUpTime = 2.5;
    }

}
