package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff
import edu.wpi.first.wpilibj.GenericHID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class shooter {
  //constants
  private static double CAM_ERROR = 1;           // Allowed aiming error in degrees
  private static double TURRET_ROTATE_KP = 1.0 / 20.0;   // Proportional constant for turret rotate speed
  private static double FLYWHEEL_VOLTAGE = 11;   // Maximum controller voltage for voltage compensation
  private static double FLYWHEEL_P = 0.75;
  private static double FLYWHEEL_I = 0.0;
  private static double FLYWHEEL_D = 100.0;
  private static double FLYWHEEL_F = 0.051;
  private static double STARTING_FLYWHEEL_SPEED = 2700;
  private static double RPM_TO_TICKS_MS = 2048.0/600.0;  // Conversion factor for rotational velocity
  private static double TRIGGER_MOTOR_SPEED = 0.4;       // Maximum power for the motor feeding the flywheel
  private static double SHOOTING_RPM_RANGE = 20;         // Allowed RPM error for flywheel
  //
  private static double CAMERA_HEIGHT = 16.0;            // Limelight height above ground (inches)
  private static double CAMERA_ANGLE  = 25.0;            // Limelight camera angle above horizontal (degrees)
  private static double TARGET_HEIGHT = 98.25;           // Center of target above ground (inches)
  private static double TARGET_HEIGHT_DELTA = TARGET_HEIGHT - CAMERA_HEIGHT;
  //
  private static double MANUAL_POWER = 0.5;             // Turret power for manual control
  //
  private static double TURRET_TICKS_PER_DEGREE = 1770 / 90;  // Encoder ticks per degree

  // Motor controllers
  public WPI_TalonSRX turretRotate;                     // Motor for rotating the turret
  private WPI_TalonSRX collectorBelt;                   // Motor for ball feed belt
  private WPI_TalonSRX triggerMotor;                    // Motor for ball loading to the flywheel (fire!)
  private WPI_TalonFX flyWheel;                         // High speed flywheel motor
  // Network Table entries
  private NetworkTableEntry tx;                         // Angle error (x) from LimeLight camera
  private NetworkTableEntry ty;                         // Angle error (y) from LimeLight camera
  private NetworkTableEntry ta;                         // Target area measurement from LimeLight camera
  private NetworkTableEntry tv;                         // Target valid indicator from Limelight camera
  // Shared variables
  public boolean targetValid;     // Indicate when the Limelight camera has found a target
  public boolean targetLocked;    // Indicate when the turret is centered on the target
  public double  targetRange;     // Range from robot to target (inches)
  public boolean flyWheelReady;   // Indicate when flywheel velocity is acceptable
  //Private autoaim variables
   private double  turretSpeed;
   private double  xError;
   private double  yError;
   private double  area;
   private boolean autoAimEnabled;

  public static boolean autonomousEnabled;
  
  /**
   * This constructor will intialize the motors and internal variables for the robot turret
   */
  public shooter() {
    turretRotate      = new WPI_TalonSRX(config_hw.turretRotateCAN);
    collectorBelt     = new WPI_TalonSRX(config_hw.ballProcessCAN);
    triggerMotor      = new WPI_TalonSRX(config_hw.ballTriggerCAN);
    flyWheel          = new WPI_TalonFX(config_hw.turretLaunchCAN);

    // Reset the turret rotation encoder
    turretRotate.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    turretRotate.setSelectedSensorPosition(0);

    //settings for flywheel constant velocity mode
    flyWheel.configFactoryDefault();            // Load known defaults for all controller values
    flyWheel.enableVoltageCompensation(true);   // Enable voltage compensation
    //keep voltage of the motor constant even if battery voltage decrease
    flyWheel.configVoltageCompSaturation(FLYWHEEL_VOLTAGE);
    flyWheel.setInverted(true);                 // Flywheel runs backwards without this
    flyWheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    flyWheel.config_kP(0, FLYWHEEL_P);
    flyWheel.config_kI(0, FLYWHEEL_I);
    flyWheel.config_kD(0, FLYWHEEL_D);
    flyWheel.config_kF(0, FLYWHEEL_F);
    flyWheel.configClosedloopRamp(1);

    //set up networktables for limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    // Establish initial values for variables we share
    targetValid   = false;
    targetLocked  = false;
    targetRange   = 0.0;
    flyWheelReady = false;
  }


  /**
   * Converts the RPM input to the ticks/100ms velocity metric used by the
   * Falcon 500 motor
   * 
   * @param rpm Rotational velocity in Revolutions Per Minute (RPM)
   * @return Encoder ticks per 100msec
   */
  public double rpmToFalcon(double rpm){
    return rpm * RPM_TO_TICKS_MS;
  }


  /**
   * Converts the internal Falcon 500 velocity measurement of ticks per 100msec
   * into Revolutions Per Minute (RPM)
   * @param falcon Rotational velocity in ticks/100msec.  This is usually read from the controller
   * @return Rotational velocity in RPM
   */
  public double falconToRPM(double falcon){
    return falcon / RPM_TO_TICKS_MS;
  }

  /**
   * Read the target x error from the Limelight camera and move the turret until the error is 0
   * 
   * @param ballShooterController Used to enable autoAim turret motion
   */
  public void autoAim() {

    // Read the Limelight data from the Network Tables
    xError      = tx.getDouble(0.0);
    yError      = ty.getDouble(0.0);
    area        = ta.getDouble(0.0);
    targetValid = (tv.getDouble(0.0) != 0); // Convert the double output to boolean

    // Compute range to target.
    // Formula taken from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    targetRange = TARGET_HEIGHT_DELTA / Math.tan(Math.toRadians(CAMERA_ANGLE + yError));
    
    // Setting power based on the xError causes the turret to slow down as the error approaches 0
    // This prevents the turret from overshooting 0 and oscillating back and forth
    // KP is a scaling factor that we tested
    turretSpeed = xError * TURRET_ROTATE_KP;

    //post driver data to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", xError);
    SmartDashboard.putNumber("LimelightY", yError);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Target Range", targetRange);
    SmartDashboard.putBoolean("Target Valid", targetValid);
    SmartDashboard.putBoolean("Target Locked", targetLocked);
  }

/**move turret to drive x to be less than "CAM_ERROR"*/
  public void moveTurret(){ 
    //x = 0 when the camera sees the target is in the center
    // Only allow the turret to track when commanded
    if (Math.abs(xError) < CAM_ERROR) {               // Turret is pointing at target (or no target)
      targetLocked = targetValid;                     // We are only locked when targetValid
      turretRotate.set(ControlMode.PercentOutput, 0); // Stop motor
    }
    else {
      targetLocked = false; //moving turret to lock onto target
      turretRotate.set(ControlMode.PercentOutput, turretSpeed);
    }
  }
  /** turret can be controlled by autoaim and driver
   * when target isn't locked can use manual aim*/
  public void teleopmoveTurret(XboxController ballshootController){
    if (ballshootController.getBumper(GenericHID.Hand.kLeft)){
      this.moveTurret();
    }
    else{
      double turretRotation = ballshootController.getX(GenericHID.Hand.kLeft) * MANUAL_POWER; //manual turret movement
      turretRotate.set(ControlMode.PercentOutput, turretRotation);

    }
    
  }

  /**
   * Publish turret rotation angle on Smart Dashboard
   */
  public void postTurretAngle() {
    double turretAngle;

    turretAngle = turretRotate.getSelectedSensorPosition() / TURRET_TICKS_PER_DEGREE;
    SmartDashboard.putNumber("Turret Angle", turretAngle);
  }


  /**
   * Uses flywheel initial speed to bring it up to desired shooting speed
   * Indicates whether flyWheelReady is true or not
   * @param ballShooterController Used to control when to shoot
   */
  public void startFlywheel(){
    double flyWheelSetVelocity;
    double flyWheelVelocity;

    // Get flywheel setpoint RPM from Smart Dashboard.  This will allow drivers to adjust, if desperate
    flyWheelSetVelocity = SmartDashboard.getNumber("Flywheel RPM", STARTING_FLYWHEEL_SPEED);
    flyWheel.set(ControlMode.Velocity, rpmToFalcon(flyWheelSetVelocity));

    // Read the current (actual) flywheel velocity from the motor controller
    flyWheelVelocity = falconToRPM(flyWheel.getSelectedSensorVelocity());

    // Check to see if the flywheel velocity is within the allowed speed range
    if (Math.abs(flyWheelVelocity - flyWheelSetVelocity) <= SHOOTING_RPM_RANGE)
      flyWheelReady = true;
    else
      flyWheelReady = false;

    // Publish our flywheel data to Smart Dashboard
    SmartDashboard.putNumber("Current Velocity", flyWheelVelocity);
    SmartDashboard.putNumber("Velocity Setpoint", flyWheelSetVelocity);
    SmartDashboard.putBoolean("Flywheel Ready", flyWheelReady);

  }
  /**Shoots ball if flywheel is ready */
  public void shootBall(){
    //Only allow the shooter to fire if the flywheel is ready
    // We could also add a check here for targetLocked and range to target (targetRange)
    //Turns on other motors needed to shoot the ball
    if (flyWheelReady) {
      collectorBelt.set(ControlMode.PercentOutput, 1);
      triggerMotor.set(ControlMode.PercentOutput, TRIGGER_MOTOR_SPEED);
    }
  }
  /**Stops motors used to shoot ball*/
  public void stopBall(){
    collectorBelt.set(ControlMode.PercentOutput, 0);
    triggerMotor.set(ControlMode.PercentOutput, 0);
  } 
  /**Reverses motors used to move ball to flywheel*/
  public void unjamBall(){
    collectorBelt.set(ControlMode.PercentOutput, -1);
    triggerMotor.set(ControlMode.PercentOutput, -TRIGGER_MOTOR_SPEED);
  }
  /**
   * Driver controlled shooting mechanisms
   * unjam ball, shoot ball, stop ball
   */
  public void teleopBall(XboxController ballshootController){
    if (ballshootController.getXButton()){
      this.unjamBall();
    }
    else if (ballshootController.getTriggerAxis(GenericHID.Hand.kRight) == 1){
      this.shootBall();
    }
    else {
      this.stopBall();
    } 
  }

}
