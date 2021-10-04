// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.PowerDistributionPanel; idk what this is but im keeping it
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff
import edu.wpi.first.wpilibj.GenericHID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.config_hw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //this defines "driverController" as an object
  public XboxController driverController;
  public WPI_TalonFX frontLeft;
  public WPI_TalonFX frontRight;
  public WPI_TalonFX rearLeft;
  public WPI_TalonFX rearRight;
  public WPI_TalonSRX turretRotate;
  private final double CAM_ERROR = 1;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //Create the primary controller object
    driverController  = new XboxController(0); 
    frontLeft         = new WPI_TalonFX(config_hw.leftFrontCAN); 
    rearLeft          = new WPI_TalonFX(config_hw.leftBackCAN);
    frontRight        = new WPI_TalonFX(config_hw.rightFrontCAN);
    rearRight         = new WPI_TalonFX(config_hw.rightBackCAN);
    turretRotate      = new WPI_TalonSRX(config_hw.turretRotateCAN);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //creating tow variables to use for the left and right on the controller
    double leftPower;
    double rightPower;
    boolean reverseDrive;
    double turretSpeed;
    //defining leftPower and rightPower to the controller
    leftPower = driverController.getY(GenericHID.Hand.kLeft) * 0.3;
    rightPower = driverController.getY(GenericHID.Hand.kRight) * 0.3;
    reverseDrive = driverController.getBumper(GenericHID.Hand.kLeft);
    
    //this code names our left and right variables in the SmartDashboard program
    SmartDashboard.putNumber("left Power", leftPower);
    SmartDashboard.putNumber("Right Power", rightPower);
    SmartDashboard.putBoolean("reverse Drive", reverseDrive);

      if (reverseDrive == true){
      // reverse drive
        frontRight.set(ControlMode.PercentOutput, -leftPower);
        rearRight.set(ControlMode.PercentOutput, -leftPower);
      //
        frontLeft.set(ControlMode.PercentOutput, rightPower);
        rearLeft.set(ControlMode.PercentOutput, rightPower);
      } else {
      //Normal drive
        frontLeft.set(ControlMode.PercentOutput, -leftPower);
        rearLeft.set(ControlMode.PercentOutput, -leftPower);
      //left values are negative to go forward
        frontRight.set(ControlMode.PercentOutput, rightPower);
        rearRight.set(ControlMode.PercentOutput, rightPower);
      }

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry tx = table.getEntry("tx");
      NetworkTableEntry ty = table.getEntry("ty");
      NetworkTableEntry ta = table.getEntry("ta");
      
      //read values periodically
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);
      double area = ta.getDouble(0.0);
      turretSpeed = x/30;
      //post to smart dashboard periodically
      SmartDashboard.putNumber("LimelightX", x);
      SmartDashboard.putNumber("LimelightY", y);
      SmartDashboard.putNumber("LimelightArea", area);

      

      if (x > CAM_ERROR) {
        //comment
        turretRotate.set(ControlMode.PercentOutput, turretSpeed);
      } else if (x < -CAM_ERROR) {
        //comment
        turretRotate.set(ControlMode.PercentOutput, turretSpeed);
      } else {
        //comment
        turretRotate.set(ControlMode.PercentOutput, 0);
      }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
