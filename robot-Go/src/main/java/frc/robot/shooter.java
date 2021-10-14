package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff
import edu.wpi.first.wpilibj.GenericHID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.config_hw;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class shooter {

    //constants
    private final double CAM_ERROR = 1;
    private final double FLYWHEEL_VOLTAGE = 11;
    private final double FLYWHEEL_P = 0.75;
    private final double FLYWHEEL_I = 0.0;
    private final double FLYWHEEL_D = 100.0;
    private final double FLYWHEEL_F = 0.051;
    private final double STARTING_FLYWHEEL_SPEED = 2700;
    private final double RPM_TO_TICKS_MS = 2048.0/600.0;
    private final double TRIGGER_MOTOR_SPEED = 0.4;
    private final double SHOOTING_RANGE = 20;

    public WPI_TalonSRX turretRotate;
    private WPI_TalonSRX collectorBelt;
    private WPI_TalonSRX triggerMotor;
    private WPI_TalonFX flyWheel;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    
    

    //the constructer will initilaize the variables

    public shooter(){
        turretRotate      = new WPI_TalonSRX(config_hw.turretRotateCAN);

        collectorBelt     = new WPI_TalonSRX(config_hw.ballProcessCAN);
        triggerMotor      = new WPI_TalonSRX(config_hw.ballTriggerCAN);
        flyWheel          = new WPI_TalonFX(config_hw.turretLaunchCAN);
        
        //settings for flywheel constant velocity mode
        flyWheel.configFactoryDefault();
        flyWheel.enableVoltageCompensation(true);
        //keep voltage of the motor constant even if battery voltage decrease
        flyWheel.configVoltageCompSaturation(FLYWHEEL_VOLTAGE);
        flyWheel.setInverted(true);
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
    }
    /**
     * convert rpm to ticks per hundred millisecond
     * @param rpm
     * @return
     */
    public double rpmToFalcon(double rpm){
      return rpm * RPM_TO_TICKS_MS;
    }
    //tphm = rpm * 2048/600
    //600*tphm/2048 = rpm
    //tphm = falcon
    public double falconToRPM(double falcon){
      return falcon / RPM_TO_TICKS_MS;
    }

    /**
     * something 
     * unique
     */
    public void autoAim(){
        double turretSpeed;
        double x;
        double y;
        double area; //can be written as "double x, y, area;"

        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        turretSpeed = x/20; //this is our ratio on how fast the turret is allowed to move to get to its position
        
        //move turret to drive x to be less than "CAM_ERROR" 
        //x = 0 when the camera sees the target is in the center
        if (x > CAM_ERROR) {
          turretRotate.set(ControlMode.PercentOutput, turretSpeed);
        } else if (x < -CAM_ERROR) {
          turretRotate.set(ControlMode.PercentOutput, turretSpeed);
        } else { // stop turret when x is between + and - CAM_ERROR
          turretRotate.set(ControlMode.PercentOutput, 0);
        }

    }

    /**
     * shoot the ball
     */
    public void ballShooter(XboxController ballShooterController){
      double flyWheelVelocity;
      double ballInsert;
      ballInsert = ballShooterController.getTriggerAxis(GenericHID.Hand.kRight);


      flyWheel.set(ControlMode.Velocity, rpmToFalcon(STARTING_FLYWHEEL_SPEED));
      flyWheelVelocity = falconToRPM(flyWheel.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Velocity in RPM", flyWheelVelocity);
      SmartDashboard.putNumber("Velocity Setpoint", STARTING_FLYWHEEL_SPEED);

      if ((ballInsert == 1) & (Math.abs(flyWheelVelocity - STARTING_FLYWHEEL_SPEED) <= SHOOTING_RANGE)){
        collectorBelt.set(ControlMode.PercentOutput, 1);
        triggerMotor.set(ControlMode.PercentOutput, TRIGGER_MOTOR_SPEED);  
      }
      else{
        collectorBelt.set(ControlMode.PercentOutput, 0);
        triggerMotor.set(ControlMode.PercentOutput, 0);
      }



    }
    
    
}
