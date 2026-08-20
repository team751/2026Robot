// DISABLED for the 2026 season — the whole climber mechanism (this file and
// ClimberSubsystem.java) is commented out. CAN IDs for the climber motors are also commented
// out in frc/robot/util/Constants.java. To re-enable: uncomment both files, restore the motor
// IDs in Constants.java, and re-add the ClimberSubsystem reference in Superstructure.java.
// package frc.robot.subsystems.climber;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import frc.lib.CTREConfig;
// import frc.robot.Robot;
// import frc.robot.util.Constants;

// public class ClimberConstants {
//   // TODO: Set current limits and tune the motors. also name/find the canbus to put the motors on
//   public static double averageMotorError = 0.3; // might remove??

//   public static final CTREConfig<TalonFX, TalonFXConfiguration> leftClimberConfig =
//       new CTREConfig<>(TalonFXConfiguration::new);
//   public static final CTREConfig<TalonFX, TalonFXConfiguration> rightClimberConfig =
//       new CTREConfig<>(TalonFXConfiguration::new);

//   public static final int kServoPort = 0;
//   public static final int rightServoPort = Constants.rightServoPort;
//   public static final int leftServoPort = Constants.leftServoPort;

//   // Rotations per second at full speed (1.0). Tune by running full speed for
//   // 1 second and comparing getServoPosition() to actual shaft rotations.
//   public static final double kServoMaxRPS = 1.0;

//   static {
//     leftClimberConfig
//         .withName("Left Climber")
//         .withCanID(Constants.leftClimberMotorID)
//         .withBus(Robot.riobus);

//     TalonFXConfiguration leftConfig = leftClimberConfig.config;
//     leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

//     leftConfig.Slot0.kP = 0.0;
//     leftConfig.Slot0.kI = 0.0;
//     leftConfig.Slot0.kD = 0.0;

//     leftConfig.Slot0.kS = 0.0;
//     leftConfig.Slot0.kV = 0.0;
//     leftConfig.Slot0.kG = 0.0;
//     leftConfig.Slot0.kA = 0.0;
//     // leftClimberConfig.CurrentLimits.StatorCurrentLimit = 150;
//     // leftClimberConfig.CurrentLimits.StatorCurrentLimitEnable = false;
//     // leftClimberConfig.TorqueCurrent.PeakForwardTorqueCurrent = 45;
//     // leftClimberConfig.TorqueCurrent.PeakReverseTorqueCurrent = -45;
//   }

//   static {
//     rightClimberConfig
//         .withName("Right Climber")
//         .withCanID(Constants.rightClimberMotorID)
//         .withBus(Robot.riobus);

//     TalonFXConfiguration rightConfig = rightClimberConfig.config;
//     rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

//     rightConfig.Slot0.kP = 0.0;
//     rightConfig.Slot0.kI = 0.0;
//     rightConfig.Slot0.kD = 0.0;

//     rightConfig.Slot0.kS = 0.0;
//     rightConfig.Slot0.kV = 0.0;
//     rightConfig.Slot0.kG = 0.0;
//     rightConfig.Slot0.kA = 0.0;
//     // rightClimberConfig.CurrentLimits.StatorCurrentLimit = 150;
//     // rightClimberConfig.CurrentLimits.StatorCurrentLimitEnable = false;
//     // rightClimberConfig.TorqueCurrent.PeakForwardTorqueCurrent = 35;
//     // rightClimberConfig.TorqueCurrent.PeakReverseTorqueCurrent = -35;
//   }
// }
