package frc.robot.commands;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.ExampleSubsystem;

public class CommandHandsOn extends Command {
    public CommandHandsOn(ExampleSubsystem subsystem) {
    addRequirements(subsystem);
    }
    public void withTimeout(int seconds) {
        
    }
}