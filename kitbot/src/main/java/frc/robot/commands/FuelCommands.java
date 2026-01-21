package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.fuel.Fuel;

public class FuelCommands {
    private Fuel fuel;

    public FuelCommands(Fuel fuel) {
        this.fuel = fuel;
    }

    // Command to intake fuel - runs while held
    public Command intake() {
        return Commands.startEnd(() -> fuel.intake(), () -> fuel.stop(), fuel).withName("fuel intake");
    }

    // Command to eject fuel (reverse intake) - runs while held
    public Command eject() {
        return Commands.startEnd(() -> fuel.eject(), () -> fuel.stop(), fuel).withName("fuel eject");
    }

    // Command to spin up the launcher (launcher at full, feeder at reduced power)
    // Does not stop on end - designed to be followed by launch()
    public Command spinUp() {
        return Commands.runOnce(() -> fuel.spinUp(), fuel).withName("fuel spin up");
    }

    // Command to launch fuel - runs while held
    // Does not stop on end - designed to be used in sequence after spinUp()
    public Command launch() {
        return Commands.run(() -> fuel.launch(), fuel).withName("fuel launch");
    }

    // Command to stop the fuel subsystem
    public Command stop() {
        return Commands.runOnce(() -> fuel.stop(), fuel).withName("fuel stop");
    }

    // Launch sequence: spin up for configured time, then launch
    // Stops when released
    public Command launchSequence() {
        return Commands.sequence(spinUp(), Commands.waitSeconds(Fuel.Constants.spinUpSeconds), launch())
                .finallyDo(() -> fuel.stop())
                .withName("fuel launch sequence");
    }
}
