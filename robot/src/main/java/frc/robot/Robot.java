// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController; // imported new library for controller
import edu.wpi.first.wpilibj.GenericHID; //library needed for leftPower and rightPower

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.config_hw;

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
// 
public XboxController driverController; //created name for object
public WPI_TalonFX frontLeft; //creating names of motors
public WPI_TalonFX frontRight;
public WPI_TalonFX rearLeft;
public WPI_TalonFX rearRight;

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
  public void teleopInit( //Init only runs once, only declares these things once
  ) {
    driverController = new XboxController(0);

    frontLeft = new WPI_TalonFX(config_hw.leftFrontCAN);
    frontRight = new WPI_TalonFX(config_hw.rightFrontCAN);
    rearLeft = new WPI_TalonFX(config_hw.leftBackCAN);
    rearRight = new WPI_TalonFX(config_hw.rightBackCAN);
  }

  /** This function is called periodically during operator control. */
  @Override
  // Create the primary driver controller object
  //Created object
  public void teleopPeriodic() { //teloepPeriodic runs infinite times
    double leftPower; //declare variable leftPower
    double rightPower; //declare variable rightPower

    leftPower = driverController.getY(GenericHID.Hand.kLeft); //assigned leftPower to the left knob and Y axis
    rightPower = driverController.getY(GenericHID.Hand.kRight); //assigned rightPower to the right knob and Y axis

    //use this to debug or check data
    //left value = lable on smart dashboard
    //right value = actual value
    SmartDashboard.putNumber("Left Power", leftPower); 
    SmartDashboard.putNumber("Right Power", rightPower);

    //setting motor power
    frontLeft.set(ControlMode.PercentOutput, -leftPower); //inverted because of motor placement on hw
    rearLeft.set(ControlMode.PercentOutput, -leftPower);  //inverted because of motor placement on hw
    
    frontRight.set(ControlMode.PercentOutput, rightPower);
    rearRight.set(ControlMode.PercentOutput, leftPower);


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
