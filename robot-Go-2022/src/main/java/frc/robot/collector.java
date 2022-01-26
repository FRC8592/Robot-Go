///////////////////////////////////////////////////////////////////////////////////////////////////////
// Manage the robot ball collector and associated pneumatics
//////////////////////////////////////////////////////////////////////////////////////////////////////

package frc.robot;

// Smart Dashboard and controller classes
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

// Pneumatic control classes
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

// Motor control classes
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class collector {
    // Pneumatics and collector position
    private Compressor     robotCompressor;
    private DoubleSolenoid collectorSolenoid;
    private boolean        collectorInboard = true;

    // Intake spin motor.  Configure for velocity PID control
    private static double INTAKE_VOLTAGE = 11;   // Maximum controller voltage for voltage compensation
    private static double INTAKE_P = 0.25;
    private static double INTAKE_I = 0.0;
    private static double INTAKE_D = 0.0;
    private static double INTAKE_F = 0.051;
    private WPI_TalonSRX  intakeSpin;

    // Parameters for collector speed
    private static final double GEAR_RATIO     = 4.0;       // Motor-to-wheel gearing
    private static final double WHEEL_DIAMETER = 0.1016;    // 4 inches = 0.1524 meters
    private static final double TICKS_PER_REV  = 1024.0;    // Redline encoder pulses per revolution
    //
    private static final double TICKS_TO_MS = (1 / (TICKS_PER_REV * GEAR_RATIO)) * WHEEL_DIAMETER * Math.PI;
    private static final double MS_TO_TICKS = 1 / TICKS_TO_MS;
    //
    private static final double INTAKE_UNJAM_MS = 1.0;      // Unjam wheel velocity in m/s

    /**
     * This constructor will initialize hardware and variables for the collector
     */
    public collector() {
        // Create pneumatic controller objects
        robotCompressor    = new Compressor(config_hw.compressorCAN, PneumaticsModuleType.CTREPCM);
        collectorSolenoid  = new DoubleSolenoid(config_hw.compressorCAN, PneumaticsModuleType.CTREPCM, config_hw.intakeSolPortA, config_hw.intakeSolPortB);

        // Initial pneumatic configuration
        robotCompressor.enableDigital();                        // Start compressor running
        //robotCompressor.disable();                              // Uncomment this line to test without compressor
        collectorSolenoid.set(DoubleSolenoid.Value.kReverse);   // Move collector inboard
        collectorInboard = true;

        // Create motor
        intakeSpin = new WPI_TalonSRX(config_hw.intakeSpinCAN);

        // Settings for intake constant velocity mode
        intakeSpin.configFactoryDefault();              // Load known defaults for all controller values
        intakeSpin.enableVoltageCompensation(true);     // Enable voltage compensation
        intakeSpin.configVoltageCompSaturation(INTAKE_VOLTAGE);
        //intakeSpin.setInverted(true);                 // TODO: Is this needed?
        intakeSpin.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
        intakeSpin.setSensorPhase(true);                // Inverts sensor phase 
        intakeSpin.config_kP(0, INTAKE_P);
        intakeSpin.config_kI(0, INTAKE_I);
        intakeSpin.config_kD(0, INTAKE_D);
        intakeSpin.config_kF(0, INTAKE_F);
        intakeSpin.configClosedloopRamp(1);
        intakeSpin.set(ControlMode.Velocity, 0);
    }


    /**
     * Operate collector pneumatics and motor
     * @param driveTrainController Raise and lower collector.  Reverse mechanism to unjam
     */
    public void collectorPeriodic(driveTrain drive, XboxController driveTrainController) {
        double wheelSpeedMSActual;
        double intakeSpeedMSActual;
        double intakeSpeedMSSet;

        //
        // Get the actual drivetrain and intake wheel speed and display on Smart Dashboard
        // Set the collector speed based on drivetrain speed
        //
        wheelSpeedMSActual  = drive.getDriveSpeed();
        intakeSpeedMSActual = getIntakeSpeed();
        intakeSpeedMSSet    = wheelSpeedMSActual + 1;     // Run the intake slightly faster than the drive wheels.
        SmartDashboard.putNumber("Wheel m/s",  wheelSpeedMSActual);
        SmartDashboard.putNumber("Intake m/s", intakeSpeedMSActual);
        SmartDashboard.putNumber("Intake Set m/s", intakeSpeedMSSet);

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
            setIntakeSpeed(-INTAKE_UNJAM_MS);     // Reverse motor to unjam
        } else if (!collectorInboard) {
            setIntakeSpeed(intakeSpeedMSSet);           // Run collector when outboard (speed relative to chassis wheel speed)
        } else {
            setIntakeSpeed(0);                          // Stop motor when inboard
        }
    }

    /**
     * Set the intake speed
     * @param wheelSpeedMS Collector wheel speed in m/s.  Generally should be a bit faster than chassis wheel speed
     */
    public void setIntakeSpeed(double wheelSpeedMS) {
        double motorTPHundy;

        // Talon XRS measures speed in ticks per 100ms.  Divide by 10 to convert per-second values
        motorTPHundy = wheelSpeedMS * MS_TO_TICKS / 10.0;

        intakeSpin.set(ControlMode.Velocity, motorTPHundy);
        
    }


    /**
    * Returns average wheel speed in m/s
    * Speed is derived from motor RPM and does not account for wheel slippage
    * @return
    * Wheel speed in m/s (double)
    */
    public double getIntakeSpeed(){
        double wheelSpeedMS;        // wheel speed in m/s

        // Talon SRX reports ticks per 100ms.  Multiple by 10 to get ticks per second.
        wheelSpeedMS = intakeSpin.getSelectedSensorVelocity() * TICKS_TO_MS * 10;

        return wheelSpeedMS;
    }

}
