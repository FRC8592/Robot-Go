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

public class driveTrain {
    public WPI_TalonFX frontLeft;
    public WPI_TalonFX frontRight;
    public WPI_TalonFX rearLeft;
    public WPI_TalonFX rearRight;

    public driveTrain(){
        frontLeft         = new WPI_TalonFX(config_hw.leftFrontCAN); 
        rearLeft          = new WPI_TalonFX(config_hw.leftBackCAN);
        frontRight        = new WPI_TalonFX(config_hw.rightFrontCAN);
        rearRight         = new WPI_TalonFX(config_hw.rightBackCAN);
    }

    public void driveTrainPeriodic(XboxController driveTrainController){
        //creating variables
        double leftPower;
        double rightPower;
        boolean reverseDrive;

    
        //defining leftPower and rightPower to the controller
        leftPower = driveTrainController.getY(GenericHID.Hand.kLeft) * 0.3;
        rightPower = driveTrainController.getY(GenericHID.Hand.kRight) * 0.3;
        reverseDrive = driveTrainController.getBumper(GenericHID.Hand.kLeft);
    
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
    }
    
}