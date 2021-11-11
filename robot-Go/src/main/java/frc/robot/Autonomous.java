package frc.robot;

import edu.wpi.first.wpilibj.Timer;


public class Autonomous {
    private enum AutoState{SHOOT, DRIVE, STOP};
    private AutoState currentState = AutoState.SHOOT;
    private Timer autonomousTimer;
    private shooter turretLauncher;
    private driveTrain drive;
    
    public Autonomous(shooter turretLauncher, driveTrain drive) {
        this.turretLauncher = turretLauncher;
        this.drive = drive;
        autonomousTimer = new Timer();

        autonomousTimer.start();

    }

    public void autoPeriodic() {
        switch(currentState) {
            case SHOOT: 
                turretLauncher.autoAim();      //turns turret towards target
                turretLauncher.moveTurret();
                turretLauncher.startFlywheel(); 
                turretLauncher.shootBall();
                if (autonomousTimer.get() >= 6){
                    autonomousTimer.reset();
                    turretLauncher.stopBall();
                    currentState = AutoState.DRIVE;
                }
                break;

            case DRIVE:
                drive.autoDrive();
                if (autonomousTimer.get() >= 1){
                    autonomousTimer.reset();
                    currentState = AutoState.STOP;
                }
                break;
            
            case STOP:
                drive.driveStop();
                break;
        }

    }
}
