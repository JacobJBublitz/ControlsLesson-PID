package org.frcteam2910.o2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;
import org.frcteam2910.o2019.subsystems.ArmSubsystem;
import org.frcteam2910.o2019.subsystems.DrivetrainSubsystem;

public class Robot extends TimedRobot {
    private static final double UPDATE_DT = 5.0e-3;

    private static final OI OI = new OI();

    private final SubsystemManager subsystemManager = new SubsystemManager(
            ArmSubsystem.getInstance(),
            DrivetrainSubsystem.getInstance()
    );

    public static OI getOi() {
        return OI;
    }

    @Override
    public void robotInit() {
        OI.bindButtons();

        subsystemManager.enableKinematicLoop(UPDATE_DT);
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();

        subsystemManager.outputToSmartDashboard();
    }
}
