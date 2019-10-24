package org.frcteam2910.o2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.o2019.Robot;
import org.frcteam2910.o2019.subsystems.DrivetrainSubsystem;

public class DriveCommand extends Command {
    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        double forward = Robot.getOi().getForwardAxis().get(true);
        double strafe = Robot.getOi().getStrafeAxis().get(true);
        double rotation = Robot.getOi().getRotationAxis().get(true);

        DrivetrainSubsystem.getInstance().holonomicDrive(new Vector2(forward, strafe), rotation, true);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
