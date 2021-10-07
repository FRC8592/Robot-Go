package frc.robot;

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

public class shooter {

    public WPI_TalonSRX turretRotate;
    private final double CAM_ERROR = 1;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

    //the constructer will initilaize the variables

    public shooter(){
        turretRotate      = new WPI_TalonSRX(config_hw.turretRotateCAN);
        //set up networktables for limelight
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }

    /**
     * something 
     * unique
     */
    public void autoaim(){
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

        turretSpeed = x/27; //this is our ratio on how fast the turret is allowed to move to get to its position
        
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
}
