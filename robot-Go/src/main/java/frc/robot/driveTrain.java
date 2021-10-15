package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class driveTrain {
  // Constants
  private static final double DRIVE_POWER = 1.0;    // Forward/reverse power scaling
  private static final double TURN_POWER  = 1.0;    // Turning power scaling
  private static final double RAMP_TIME   = 0.5;    // Smooth application of motor power

  // Motor controllers
  public WPI_TalonFX frontLeft;
  public WPI_TalonFX frontRight;
  public WPI_TalonFX rearLeft;
  public WPI_TalonFX rearRight;

  // Motor groups
	SpeedControllerGroup leftDrive;
	SpeedControllerGroup rightDrive;
	  
	// Differential drive class
  DifferentialDrive robotDrive;


  /**
   * Initialize drivetrain
   */
  public driveTrain(){
    // Create motor objects
    frontLeft  = new WPI_TalonFX(config_hw.leftFrontCAN); 
    rearLeft   = new WPI_TalonFX(config_hw.leftBackCAN);
    frontRight = new WPI_TalonFX(config_hw.rightFrontCAN);
    rearRight  = new WPI_TalonFX(config_hw.rightBackCAN);

    // Configure motor ramp time to smooth out acceleration
		frontLeft.configOpenloopRamp(RAMP_TIME);
		rearLeft.configOpenloopRamp(RAMP_TIME);
		frontRight.configOpenloopRamp(RAMP_TIME);
	  rearRight.configOpenloopRamp(RAMP_TIME);
	
		// Pair up motors into control groups
		leftDrive  = new SpeedControllerGroup(frontLeft, rearLeft);
		rightDrive = new SpeedControllerGroup(frontRight, rearRight);
		  
		// Initialize drive system
  	robotDrive = new DifferentialDrive(rightDrive, leftDrive);
  }


  public void driveTrainPeriodic(XboxController driveTrainController){
    double  forward;
    double  reverse;
    double  throttle;
    double  turn;
    boolean reverseControl;
    boolean curveOff;

    // Read gamepad controls
    forward = driveTrainController.getTriggerAxis(GenericHID.Hand.kRight);  // Right trigger
    reverse = driveTrainController.getTriggerAxis(GenericHID.Hand.kLeft);   // Left Trigger
    turn    = driveTrainController.getX(GenericHID.Hand.kLeft);             // Left joystick
    //
    reverseControl = driveTrainController.getBButton();                      // B button
    curveOff       = driveTrainController.getBumper(GenericHID.Hand.kRight); // Right bumper
  
    // Combine and scale inputs
    throttle = (-forward + reverse) * DRIVE_POWER;
    turn     = turn * TURN_POWER;

    // If reverseControl is being pressed, invert all inputs so the robot can be driven backwards
    if (reverseControl) {
      throttle = -throttle;
      turn     = -turn;
    }
    
    // Send controls to the robot drive system
    robotDrive.curvatureDrive(throttle, turn, curveOff);
  }

}
