package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class driveTrain {
  // Constants
  private static final double DRIVE_POWER = 1.0;              // Forward/reverse power scaling
  private static final double TURN_POWER  = 0.6;              // Turning power scaling
  private static final double TURN_IN_PLACE_POWER  = 0.45;    // Turning power scaling
  private static final double RAMP_TIME   = 0.25;             // Smooth application of motor power
  //
  private static final double GEAR_RATIO    = 10.75;    // Motor-to-wheel gearing
  private static final double WHEEL_DIAMETER = 0.1524;   // 6 inches = 0.1524 meters
  private static final double TICKS_PER_REV = 2048.0;   // Falcon 500 encoder ticks per revolution
  //
  private static final double TICKS_TO_MS   = (1 / (TICKS_PER_REV * GEAR_RATIO)) * WHEEL_DIAMETER * Math.PI;

  // Motor controllers
  private WPI_TalonFX frontLeft;
  private WPI_TalonFX frontRight;
  private WPI_TalonFX rearLeft;
  private WPI_TalonFX rearRight;

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

    // Configure encoder for each motor (used to monitor velocity)
    frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rearLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    frontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rearRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

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

/**Drives robot: forwards, backwards, turning*/
  public void driveTrainPeriodic(XboxController driveTrainController){
    double  forward;
    double  reverse;
    double  throttle;
    double  turn;
    boolean reverseControl;
    boolean curveOff;

    // Read gamepad controls
    forward = driveTrainController.getRightTriggerAxis();  // Right trigger
    reverse = driveTrainController.getLeftTriggerAxis();   // Left Trigger
    turn    = driveTrainController.getLeftX();             // Left joystick
    //
    reverseControl = driveTrainController.getBButton();                      // B button
    curveOff       = driveTrainController.getRightBumper(); // Right bumper
  
    // Combine and scale inputs
    throttle = (-forward + reverse) * DRIVE_POWER;

    // Apply a different power curve for turn-in-place
    if (!curveOff)
      turn = turn * TURN_POWER;
    else
      turn = turn * TURN_IN_PLACE_POWER;

    // If reverseControl is being pressed, invert all inputs so the robot can be driven backwards
    if (reverseControl) {
      throttle = -throttle;
    }
    
    // Send controls to the robot drive system
    robotDrive.curvatureDrive(throttle, turn, curveOff);
  }

  public void autoDrive(){
    robotDrive.curvatureDrive(0.3, 0, false);
  }

  public void driveStop(){
    robotDrive.curvatureDrive(0, 0, false);
  }

  /**
  * Returns average wheel speed in m/s
  * Speed is derived from motor RPM and does not account for wheel slippage
  * @return
  * Wheel speed in m/s (double)
  */
  public double getDriveSpeed(){
    double motorTPS;      // Average ticks per second from left and right motors
    double wheelSpeedMS;  // Wheel revolutions per second

    // Falcons report ticks per 100ms.  Multiple by 10 to get ticks per second.  Divide by two to get average of left and right
    motorTPS     = (frontLeft.getSelectedSensorVelocity() - frontRight.getSelectedSensorVelocity()) * 5.0;
    wheelSpeedMS = motorTPS * TICKS_TO_MS;
    SmartDashboard.putNumber("Chassis TPS",  motorTPS);

    return wheelSpeedMS;
  }

}
