package org.frcteam2910.o2019.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.frcteam2910.o2019.subsystems.ArmSubsystem;

public class SetArmAngleCommand extends InstantCommand {
    public SetArmAngleCommand(double targetAngle) {
        super(() -> ArmSubsystem.getInstance().setTargetAngle(targetAngle));

        requires(ArmSubsystem.getInstance());
    }
}
