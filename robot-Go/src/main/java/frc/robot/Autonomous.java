package frc.robot;

import edu.wpi.first.wpilibj.Timer;


public class Autonomous {
    private enum AutoState{SHOOT, DRIVE, STOP};
    private AutoState currentState = AutoState.SHOOT;
    private Timer autonomousTimer;
    
    public void Autonomous() {
        autonomousTimer = new Timer();

        autonomousTimer.start();
    }

    public void autoPeriodic() {
        switch(currentState) {
            case SHOOT: 

            
        }

    }
}
