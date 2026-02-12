package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.SwerveSubsystem;

import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.PS5Controller;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.simulation.MapSimSwerveTelemetry;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import frc.robot.subsystems.simulation.MapleSimSwerveDrivetrain;

/**
 *
 *
 * <h1>Represents an FUEL launched into the air.</h1>
 *
 * <p>This class models a {@link RebuiltFuelOnField} launched into the air.
 *
 * <p>The simulation will determine if the FUEL hits its target—the HUB.
 *
 * <p>The user can specify a callback using {@link #setHitNetCallBack(Runnable)}, which will be
 * triggered when the FUEL hits the HUB.
 */
public class RebuiltFuelOnFly extends GamePieceProjectile {

	public RebuiltFuelOnFly(
			Translation2d robotPosition,
			Translation2d shooterPositionOnRobot,
			ChassisSpeeds chassisSpeeds,
			Rotation2d shooterFacing,
			Distance initialHeight,
			LinearVelocity launchingSpeed,
			Angle shooterAngle) {
		super(
				RebuiltFuelOnField.REBUILT_FUEL_INFO,
				robotPosition,
				shooterPositionOnRobot,
				chassisSpeeds,
				shooterFacing,
				initialHeight,
				launchingSpeed,
				shooterAngle);				

		super.withTouchGroundHeight(Inches.of(10).in(Meters));
		super.enableBecomesGamePieceOnFieldAfterTouchGround();
	}
}