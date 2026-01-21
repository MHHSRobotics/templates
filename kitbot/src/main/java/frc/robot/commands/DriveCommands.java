package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.drive.Drive;

public class DriveCommands {
    private Drive drive;

    public DriveCommands(Drive drive) {
        this.drive = drive;
    }

    // Reset drive gyro to 0
    public Command resetGyro() {
        return new InstantCommand(() -> drive.resetGyro()).withName("drive reset gyro");
    }

    // Drives using arcade controls with the given forward and turn inputs.
    // Applies deadband and power scaling for smoother control.
    public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier turn) {
        return Commands.run(
                        () -> {
                            double fwd = forward.getAsDouble();
                            double rot = turn.getAsDouble();

                            // Apply deadband to forward input
                            fwd = MathUtil.applyDeadband(fwd, Drive.Constants.moveDeadband);
                            // Apply power scaling for smoother control
                            fwd = Math.copySign(Math.pow(Math.abs(fwd), Drive.Constants.movePow), fwd);

                            // Apply deadband to turn input
                            rot = MathUtil.applyDeadband(rot, Drive.Constants.turnDeadband);
                            // Apply power scaling for smoother control
                            rot = Math.copySign(Math.pow(Math.abs(rot), Drive.Constants.turnPow), rot);

                            // Drive with arcade controls
                            drive.arcadeDrive(fwd, rot);
                        },
                        drive)
                .withName("arcade drive");
    }

    // Drives using tank controls with the given left and right inputs.
    // Applies deadband and power scaling for smoother control.
    public Command tankDrive(DoubleSupplier left, DoubleSupplier right) {
        return Commands.run(
                        () -> {
                            double leftSpeed = left.getAsDouble();
                            double rightSpeed = right.getAsDouble();

                            // Apply deadband
                            leftSpeed = MathUtil.applyDeadband(leftSpeed, Drive.Constants.moveDeadband);
                            rightSpeed = MathUtil.applyDeadband(rightSpeed, Drive.Constants.moveDeadband);

                            // Apply power scaling
                            leftSpeed =
                                    Math.copySign(Math.pow(Math.abs(leftSpeed), Drive.Constants.movePow), leftSpeed);
                            rightSpeed =
                                    Math.copySign(Math.pow(Math.abs(rightSpeed), Drive.Constants.movePow), rightSpeed);

                            // Drive with tank controls
                            drive.tankDrive(leftSpeed, rightSpeed);
                        },
                        drive)
                .withName("tank drive");
    }

    // Command to set arcade drive speeds directly (no scaling)
    public Command setArcadeSpeed(double forward, double turn) {
        return new InstantCommand(() -> drive.arcadeDrive(forward, turn), drive).withName("drive set arcade speed");
    }

    // Command to set tank drive speeds directly (no scaling)
    public Command setTankSpeed(double left, double right) {
        return new InstantCommand(() -> drive.tankDrive(left, right), drive).withName("drive set tank speed");
    }

    // Stops all drive output
    public Command stop() {
        return new InstantCommand(() -> drive.stop(), drive).withName("drive stop");
    }
}
