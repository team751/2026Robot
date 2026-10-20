package frc.robot.util;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.PS5Controller;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JiggleCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SimpleUnstuck;
import frc.robot.commands.SpitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.simulation.MapSimSwerveTelemetry;
import frc.robot.subsystems.transfer.TransferSubsystem;
import org.littletonrobotics.junction.Logger;

// import frc.robot.subsystems.climber.ClimberSubsystem;

// TODO: shift button! \/\/\/
// controller.circleButton.and(controller.crossButton).whileTrue(
//   new StartEndCommand(() -> ShooterSubsystem.getInstance().requestShoot(), () ->
// ShooterSubsystem.getInstance().requestIdle())
// );
// we get basically double the buttons that we have now. could also help us test stuff without
// ruining previous keybinds
// could also be helpful during comp for re-zeroing components mid match if we need to (like intake
// explodes and dies and we need to align it again)

public class ControlBoard {
  private static ControlBoard instance;

  /* Controllers */
  private PS5Controller driver = null;
  private PS5Controller operator = null;
  private SwerveSubsystem drive = SwerveSubsystem.getInstance();
  private boolean preciseControl = false;
  private boolean autoAim = false;
  private boolean axisAlign = false;
  private double lastShotTime = 0.3;
  private double operatorOffset = 0;

  public boolean isBlue = false;

  /* Haptics */
  private static final double HUB_WARNING_SECONDS = 7.0;
  private static final double VISION_LOST_BUZZ_SECONDS = 6.0;

  private PIDController autoAimController = new PIDController(0.4, 0.0, 0.01);
  private PIDController axisAlignController =
      new PIDController(0.5, 0, 0.05); // , new Constraints(1, 1));

  private enum ControllerPreset {
    DRIVER(0),
    OPERATOR(1);

    private final int port;

    ControllerPreset(int port) {
      this.port = port;
    }

    public int port() {
      return port;
    }
  }

  /* Subsystems */
  private final Superstructure superstructure = Superstructure.getInstance();

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(SwerveConstants.maxSpeed * 0.05)
          .withRotationalDeadband(SwerveConstants.maxAngularSpeed * 0.1)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SwerveModule.SteerRequestType.Position)
          .withDesaturateWheelSpeeds(true)
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);

  private ControlBoard() {
    DriverStation.silenceJoystickConnectionWarning(true);
    autoAimController.enableContinuousInput(-180, 180);
    tryInit();
  }

  public void tryInit() {
    if (driver == null) {
      driver = new PS5Controller(ControllerPreset.DRIVER.port());
      configureBindings(ControllerPreset.DRIVER, driver);

      drive.setDefaultCommand(drive.applyRequest(this::getDriverRequest));
      if (Utils.isSimulation())
        drive.registerTelemetry(new MapSimSwerveTelemetry(SwerveConstants.maxSpeed)::telemeterize);
      System.out.println("Driver Initialized");
    }

    if (operator == null) {
      operator = new PS5Controller(ControllerPreset.OPERATOR.port());
      configureBindings(ControllerPreset.OPERATOR, operator);
      System.out.println("Operator Initialized");
    }
  }

  public void displayUI() {
    SmartDashboard.putBoolean("Hub/Active", HubShift.isOurHubActive());
    SmartDashboard.putBoolean("Hub/Shifts Known", HubShift.isKnown());
    SmartDashboard.putNumber("Hub/Seconds Until Active", HubShift.getSecondsUntilActive());
    SmartDashboard.putNumber("Hub/Active Seconds Left", HubShift.getSecondsLeftActive());
    SmartDashboard.putNumber("Hub/Endgame Seconds Left", HubShift.getEndgameSecondsLeft());
  }

  public static ControlBoard getInstance() {
    if (instance == null) instance = new ControlBoard();
    return instance;
  }

  private void configureBindings(ControllerPreset preset, PS5Controller controller) {
    switch (preset) {
      case DRIVER -> configureDriverBindings(controller);
      case OPERATOR -> configureOperatorBindings(controller);
      default -> throw new IllegalStateException("Unexpected value: " + preset);
    }
  }

  private static double getAxisAlignAngle(double currentDegrees) {
    currentDegrees = currentDegrees % 360;
    if (currentDegrees > 180) currentDegrees -= 360;
    if (currentDegrees < -180) currentDegrees += 360;
    return Math.abs(currentDegrees) <= 90 ? 0.0 : 180.0;
  }

  /* Driver bindings */
  private void configureDriverBindings(PS5Controller controller) {
    /* Intake */
    controller.leftTrigger.whileTrue(
        new IntakeCommand(IntakeSubsystem.getInstance(), ExtenderSubsystem.getInstance()));

    controller.crossButton.whileTrue(
        new JiggleCommand(
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance(),
            TransferSubsystem.getInstance()));
    controller.crossButton.whileTrue(controller.rumble(0.5));

    // controller.squareButton.onTrue(
    // new InstantCommand(() -> {
    //     if (LimelightConstants.MT_SWITCH_DISTANCE_METERS == 1.5) {
    //         LimelightConstants.MT_SWITCH_DISTANCE_METERS = 300.0;
    //     } else {
    //         LimelightConstants.MT_SWITCH_DISTANCE_METERS = 1.5;
    //     }
    // })
    // );

    controller.dUp.whileTrue(
        new InstantCommand(() -> ExtenderSubsystem.getInstance().requestExtend()));

    controller.dDown.whileTrue(
        new InstantCommand(() -> ExtenderSubsystem.getInstance().requestRetract()));

    controller.dDown.whileTrue(
        new StartEndCommand(
            () -> IntakeSubsystem.getInstance().requestIntake(),
            () -> IntakeSubsystem.getInstance().requestIdle()));

    controller.dLeft.whileTrue(new SimpleUnstuck(TransferSubsystem.getInstance()));

    /* Shooter */
    controller.rightTrigger.whileTrue(
        new ShootCommand(ShooterSubsystem.getInstance(), TransferSubsystem.getInstance()));
    controller.rightTrigger.onFalse(new InstantCommand(() -> operatorOffset = 0));
    controller.rightTrigger.whileTrue(
        new StartEndCommand(() -> autoAim = true, () -> autoAim = false)
            .withName("Auto Aim Toggle"));

    // controller.rightTrigger.whileTrue(
    //     new JiggleCommand(
    //         IntakeSubsystem.getInstance(),
    //         ExtenderSubsystem.getInstance(),
    //         TransferSubsystem.getInstance()));

    controller.rightJoystickButton.whileTrue(
        new StartEndCommand(() -> axisAlign = true, () -> axisAlign = false)
            .withName("Axis Align Toggle"));

    controller.leftBumper.whileTrue(
        new SpitCommand(
            TransferSubsystem.getInstance(),
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance()));

    controller.leftJoystickButton.whileTrue(
        new StartEndCommand(() -> axisAlign = true, () -> axisAlign = false)
            .withName("Axis Align Toggle"));

    controller.circleButton.onTrue(
        new InstantCommand(() -> drive.setRobotRotationByAlliance()).ignoringDisable(true));

    controller.rightBumper.whileTrue(
        new StartEndCommand(() -> preciseControl = true, () -> preciseControl = false)
            .withName("Precise Control Toggle")); // Fight me owen

    controller.createButton.whileTrue(
        new StartEndCommand(
                () -> Logger.recordOutput("Timestamp", true),
                () -> Logger.recordOutput("Timestamp", false))
            .ignoringDisable(true));

    /* Haptics */
    new Trigger(drive::hasCollided)
        .and(DriverStation::isTeleopEnabled)
        .onTrue(controller.rumble(drive::collisionType).withTimeout(0.3));

    new Trigger(() -> Odometry.getInstance().getSecondsWithoutVision() > VISION_LOST_BUZZ_SECONDS)
        .and(DriverStation::isTeleopEnabled)
        .onTrue(controller.rumble(1.0).withTimeout(0.6));

    configureSharedHaptics(controller);
  }

  /* Haptics used by both driver and operator */
  private void configureSharedHaptics(PS5Controller controller) {
    // Short tap when the flywheel reaches speed while shooting
    controller
        .rightTrigger
        .and(ShooterSubsystem.getInstance()::isAtSpeed)
        .onTrue(controller.rumble(0.4).withTimeout(0.15));

    // Repeating pulses in the last few seconds before our hub becomes active
    new Trigger(this::isHubAboutToActivate).whileTrue(hubWarningPulses(controller));
  }

  private boolean isHubAboutToActivate() {
    return DriverStation.isTeleopEnabled()
        && !HubShift.isOurHubActive()
        && HubShift.getSecondsUntilActive() <= HUB_WARNING_SECONDS;
  }

  private Command hubWarningPulses(PS5Controller controller) {
    return Commands.sequence(controller.rumble(0.6).withTimeout(0.2), Commands.waitSeconds(0.3))
        .repeatedly();
  }

  /* Operator bindings */
  private void configureOperatorBindings(PS5Controller controller) {
    controller.rightTrigger.whileTrue(
        new ShootCommand(ShooterSubsystem.getInstance(), TransferSubsystem.getInstance(), true));
    configureSharedHaptics(controller);

    controller.dUp.whileTrue(
        new InstantCommand(() -> ExtenderSubsystem.getInstance().requestExtend()));

    controller.dDown.whileTrue(
        new InstantCommand(() -> ExtenderSubsystem.getInstance().requestRetract()));

    controller.dRight.whileTrue(new InstantCommand(() -> operatorOffset += isBlue ? -5.0 : 5.0));
    controller.dLeft.whileTrue(new InstantCommand(() -> operatorOffset += isBlue ? 5.0 : -5.0));
    // Light tick so the operator can feel each offset step
    controller.dRight.onTrue(controller.rumble(0.3).withTimeout(0.1));
    controller.dLeft.onTrue(controller.rumble(0.3).withTimeout(0.1));

    controller.crossButton.whileTrue(
        new JiggleCommand(
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance(),
            TransferSubsystem.getInstance()));
    controller.crossButton.whileTrue(controller.rumble(0.5));
    controller.leftTrigger.whileTrue(
        new IntakeCommand(IntakeSubsystem.getInstance(), ExtenderSubsystem.getInstance()));

    controller.crossButton.whileTrue(
        new JiggleCommand(
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance(),
            TransferSubsystem.getInstance()));

    controller.createButton.whileTrue(
        new StartEndCommand(
                () -> Logger.recordOutput("Timestamp", true),
                () -> Logger.recordOutput("Timestamp", false))
            .ignoringDisable(true));

    // controller.squareButton.onTrue(
    // new InstantCommand(() -> {
    //     if (LimelightConstants.MT_SWITCH_DISTANCE_METERS == 1.5) {
    //         LimelightConstants.MT_SWITCH_DISTANCE_METERS = 300.0;
    //     } else {
    //         LimelightConstants.MT_SWITCH_DISTANCE_METERS = 1.5;
    //     }
    // }));

    /* Climber */
    // TODO: Make left trigger shoot(peter requested)
    // controller.leftTrigger.whileTrue(
    //     new StartEndCommand(() -> climber.spinSlow(1), () -> climber.stopMotors()));

    // controller.crossButton.whileTrue(
    //     new StartEndCommand(() -> climber.spinSlow(-1), () -> climber.stopMotors()));

  }

  public SwerveRequest getDriverRequest() {
    if (driver == null) return null;

    double scale = preciseControl ? 0.2 : 1.0;
    double rotScale = preciseControl ? 0.30 : 1.0;

    double rawStickRot = driver.rightHorizontalJoystick.getAsDouble();
    double rot =
        rotScale
            * SwerveConstants.maxAngularSpeed
            * (Math.copySign(rawStickRot * rawStickRot, rawStickRot));

    if (autoAim) {
      Pose2d robotPose = drive.getPose();
      Pose2d hubPose = FieldConstants.getAllianceHub();
      double angleDiff =
          Math.toDegrees(
              Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX()));
      // SmartDashboard.putNumber("target offness", angleDiff -
      // robotPose.getRotation().getDegrees());
      rot =
          autoAimController.calculate(
              robotPose.getRotation().getDegrees(), angleDiff + operatorOffset);
      // SmartDashboard.putNumber("pid value", rot);
    }

    double x = driver.leftVerticalJoystick.getAsDouble();
    double y;

    if (axisAlign) {
      Pose2d robotPose = drive.getPose();

      Pose2d nearestTrench = new Pose2d(5, 5, Rotation2d.fromDegrees(0));
      Pose2d thing = FieldConstants.getNearestTrench(robotPose, isBlue);
      // SmartDashboard.putNumber("ControlBoard/paigus", thing.getY());
      y = (isBlue ? 1.0 : -1.0) * axisAlignController.calculate(robotPose.getY(), thing.getY());
      y = Math.min(y, 1.0);
      y = Math.max(y, -1.0);
      rot =
          autoAimController.calculate(
              robotPose.getRotation().getDegrees(),
              getAxisAlignAngle(robotPose.getRotation().getDegrees()));
    } else {
      y = driver.leftHorizontalJoystick.getAsDouble();
    }

    // SmartDashboard.putBoolean("ControlBoard/AutoAim", autoAim);
    // SmartDashboard.putBoolean("ControlBoard/AxisAlign", axisAlign);
    // SmartDashboard.putNumber("ControlBoard/OperatorOffset", operatorOffset);
    // SmartDashboard.putNumber("ControlBoard/limelightdist",
    // LimelightConstants.MT_SWITCH_DISTANCE_METERS);

    return driveRequest
        .withVelocityX(SwerveConstants.maxSpeed * x * scale)
        .withVelocityY(SwerveConstants.maxSpeed * y * scale)
        .withRotationalRate(rot);
  }
}
