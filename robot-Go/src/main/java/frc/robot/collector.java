///////////////////////////////////////////////////////////////////////////////////////////////////////
// Manage the robot ball collector and associated pneumatics
//////////////////////////////////////////////////////////////////////////////////////////////////////

package frc.robot;

// Pneumatic control classes
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;

// Motor control classes
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class collector {
    // Pneumatics and collector position
    private Compressor     robotCompressor;
    private DoubleSolenoid collectorSolenoid;
    private boolean        collectorInboard = true;

    // Intake spin motor
    private WPI_TalonSRX   intakeSpin;

    /**
     * This constructor will initialize hardware and variables for the collector
     */
    public collector() {
        // Create pneumatic controller objects
        robotCompressor    = new Compressor(config_hw.compressorCAN);
        collectorSolenoid  = new DoubleSolenoid(config_hw.compressorCAN, config_hw.intakeSolPortA, config_hw.intakeSolPortB);

        // ****************************************************************************
        // *** DANGER : Closed loop control must be enabled to prevent overpressure ***
        // ****************************************************************************
        robotCompressor.setClosedLoopControl(true);             // Cycle to control pressure (important!)
        robotCompressor.start();                                // Start compressor running
        //robotCompressor.stop();                               // Uncomment this line to test without compressor
        collectorSolenoid.set(DoubleSolenoid.Value.kReverse);   // Move collector inboard
        collectorInboard = true;

        // Create motors
        intakeSpin = new WPI_TalonSRX(config_hw.intakeSpinCAN);
        intakeSpin.set(ControlMode.PercentOutput, 0);           // Ensure motor is stopped
    }


    /**
     * Operate collector pneumatics and motor
     * @param driveTrainController Raise and lower collector.  Reverse mechanism to unjam
     */
    public void collectorPeriodic(XboxController driveTrainController) {
        //
        // Move the collector inboard or outboard
        //
        if (driveTrainController.getAButton()) {
            collectorSolenoid.set(DoubleSolenoid.Value.kForward);   // Move outboard
            collectorInboard = false;
        }
        else if (driveTrainController.getYButton()) {
            collectorSolenoid.set(DoubleSolenoid.Value.kReverse);   // Move inboard
            collectorInboard = true;
        }
 
        //
        // The collector motor is automatically activated any time the collector is in the
        // outboard position.  The unjam button has priority in all situations
        //
        if (driveTrainController.getXButton()) {
            intakeSpin.set(ControlMode.PercentOutput, -1);      // Reverse motor to unjam
        } else if (!collectorInboard) {
            intakeSpin.set(ControlMode.PercentOutput, 1);       // Run collector when outboard
        } else {
            intakeSpin.set(ControlMode.PercentOutput, 0);
        }
    }
}
