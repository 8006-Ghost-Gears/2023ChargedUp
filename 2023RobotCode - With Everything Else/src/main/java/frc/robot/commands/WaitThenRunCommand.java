// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Add your docs here. */
public class WaitThenRunCommand extends InstantCommand {
    private final double delaySeconds;
    private final Runnable runnable;

    public WaitThenRunCommand(double delaySeconds, Runnable runnable) {
        super();
        this.delaySeconds = delaySeconds;
        this.runnable = runnable;
    }

    @Override
    public void initialize() {
        new WaitCommand(delaySeconds)
                .andThen(() -> runnable.run())
                .schedule();
        this.withTimeout(delaySeconds + 0.1); // add a little extra time to account for scheduler delays
    }
}

