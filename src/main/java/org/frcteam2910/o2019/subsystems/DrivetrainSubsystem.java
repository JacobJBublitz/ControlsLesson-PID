package org.frcteam2910.o2019.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.subsystems.SwerveDrivetrain;
import org.frcteam2910.o2019.RobotMap;
import org.frcteam2910.o2019.commands.DriveCommand;
import org.frcteam2910.o2019.drivers.Mk2SwerveModule;

public class DrivetrainSubsystem extends SwerveDrivetrain {
    private static final double TRACKWIDTH = 19.5;
    private static final double WHEELBASE = 23.5;

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(351.9);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(252.0);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(218.1);
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(268.9);

    private static final NetworkTableEntry FRONT_LEFT_CURRENT_ANGLE_ENTRY;
    private static final NetworkTableEntry FRONT_LEFT_TARGET_ANGLE_ENTRY;

    private static final Object INSTANCE_LOCK = new Object();
    private static DrivetrainSubsystem instance;

    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

    private final Mk2SwerveModule frontLeftModule;
    private final Mk2SwerveModule frontRightModule;
    private final Mk2SwerveModule backLeftModule;
    private final Mk2SwerveModule backRightModule;
    private SwerveModule[] swerveModules;

    static {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        FRONT_LEFT_CURRENT_ANGLE_ENTRY = tab.add("Front Left Current Angle", 0.0)
                .getEntry();
        FRONT_LEFT_TARGET_ANGLE_ENTRY = tab.add("Front Left Target Angle", 0.0)
                .getEntry();
    }

    public DrivetrainSubsystem() {
        gyroscope.calibrate();
        gyroscope.setInverted(true);

        frontLeftModule = new Mk2SwerveModule(
                new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                FRONT_LEFT_ANGLE_OFFSET,
                new Spark(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER)
        );
        frontLeftModule.setName("Front Left");

        frontRightModule = new Mk2SwerveModule(
                new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                FRONT_RIGHT_ANGLE_OFFSET,
                new Spark(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER)
        );
        frontRightModule.setName("Front Right");

        backLeftModule = new Mk2SwerveModule(
                new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                BACK_LEFT_ANGLE_OFFSET,
                new Spark(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER)
        );
        backLeftModule.setName("Back Left");

        backRightModule = new Mk2SwerveModule(
                new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                BACK_RIGHT_ANGLE_OFFSET,
                new Spark(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER)
        );
        backRightModule.setName("Back Right");

        swerveModules = new SwerveModule[]{
                frontLeftModule,
                frontRightModule,
                backLeftModule,
                backRightModule,
        };
    }

    public static DrivetrainSubsystem getInstance() {
        synchronized (INSTANCE_LOCK) {
            if (instance == null) {
                instance = new DrivetrainSubsystem();
            }

            return instance;
        }
    }

    @Override
    public Gyroscope getGyroscope() {
        return gyroscope;
    }

    @Override
    public double getMaximumVelocity() {
        return 0;
    }

    @Override
    public double getMaximumAcceleration() {
        return 0;
    }

    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    @Override
    public void outputToSmartDashboard() {
        super.outputToSmartDashboard();

        FRONT_LEFT_CURRENT_ANGLE_ENTRY.setDouble(
                Math.toDegrees(frontLeftModule.getCurrentAngle())
        );
        FRONT_LEFT_TARGET_ANGLE_ENTRY.setDouble(
                Math.toDegrees(frontLeftModule.getTargetAngle())
        );
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveCommand());
    }
}
