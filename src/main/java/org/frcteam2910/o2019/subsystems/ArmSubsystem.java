package org.frcteam2910.o2019.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.robot.subsystems.Subsystem;
import org.frcteam2910.o2019.RobotMap;

public class ArmSubsystem extends Subsystem {
    private static final double ANGLE_OFFSET = Math.toRadians(-221.75);
    private static final double MAX_ANGLE = Math.toRadians(108.26);

    private static final double ENCODER_GEAR_RATIO = 24.0 / 54.0;

    private static final ArmSubsystem INSTANCE = new ArmSubsystem();

    private static final NetworkTableEntry CURRENT_ANGLE_ENTRY;
    private static final NetworkTableEntry TARGET_ANGLE_ENTRY;

    private final SpeedController[] motors = {
            new Spark(RobotMap.ARM_MOTOR_A),
            new Spark(RobotMap.ARM_MOTOR_B)
    };

    private final AnalogInput angleEncoder = new AnalogInput(RobotMap.ARM_ENCODER);

    private final Object stateLock = new Object();
    private double currentAngle = 0.0;
    private double targetAngle = Double.NaN;

    private double lastKinematicsTimestep = Double.NaN;

    static {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        CURRENT_ANGLE_ENTRY = tab.add("Current Angle", 0.0)
                .getEntry();
        TARGET_ANGLE_ENTRY = tab.add("Target Angle", 0.0)
                .getEntry();
    }

    private ArmSubsystem() {
        // TODO: Initialize arm controller
    }

    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    public double getCurrentAngle() {
        synchronized (stateLock) {
            return currentAngle;
        }
    }

    public double getTargetAngle() {
        synchronized (stateLock) {
            return targetAngle;
        }
    }

    public void setTargetAngle(double angle) {
        angle = MathUtils.clamp(angle, 0.0, MAX_ANGLE);

        synchronized (stateLock) {
            targetAngle = angle;
        }
    }

    @Override
    protected void initDefaultCommand() {

    }

    @Override
    public void outputToSmartDashboard() {
        CURRENT_ANGLE_ENTRY.setDouble(Math.toDegrees(getCurrentAngle()));
        TARGET_ANGLE_ENTRY.setDouble(Math.toDegrees(getTargetAngle()));
    }


    @Override
    public void updateKinematics(double timestamp) {
        double dt = timestamp - lastKinematicsTimestep;
        lastKinematicsTimestep = timestamp;
        if (!Double.isFinite(dt)) {
            return;
        }

        updateSensors();

        double currentAngle = getCurrentAngle();
        double targetAngle = getTargetAngle();

        double output = 0.0;
        if (!Double.isFinite(targetAngle)) {
            output = 0.0;
        } else {
            // TODO: Do PID controller stuff
        }

        for (SpeedController sp : motors) {
            sp.set(output);
        }
    }

    private void updateSensors() {
        double encoderRotations = angleEncoder.getVoltage() / RobotController.getVoltage5V();
        double armRotations = ENCODER_GEAR_RATIO * encoderRotations;
        double armUnadjustedAngle = armRotations * 2.0 * Math.PI;
        double armAngle = 2.0 * Math.PI - armUnadjustedAngle + ANGLE_OFFSET;
        armAngle %= 2.0 * Math.PI;
        if (armAngle < 0.0) {
            armAngle += 2.0 * Math.PI;
        }

        synchronized (stateLock) {
            currentAngle = armAngle;
        }
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }


}
