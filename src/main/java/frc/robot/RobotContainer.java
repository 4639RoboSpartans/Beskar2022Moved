/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import static frc.robot.Constants.Buttons;

import java.util.List;

import javax.swing.ButtonGroup;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.ManualDriveCmd;
import frc.robot.commands.PushBallsCmd;
import frc.robot.commands.SpoolShooterCmd;
import frc.robot.commands.Turret90Cmd;
import frc.robot.commands.VisionAimCmd;
import frc.robot.subsystems.ClimberSys;
import frc.robot.subsystems.DrivetrainSys;
import frc.robot.subsystems.HopperSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.KickerSys;
import frc.robot.subsystems.PhotonVisionSys;
import frc.robot.subsystems.ShooterSys;
import frc.robot.subsystems.ShroudSys;
import frc.robot.subsystems.TurretSys;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	public final ClimberSys m_climber= new ClimberSys();
	public final DrivetrainSys m_drive= new DrivetrainSys();
	public final IntakeSys m_intake= new IntakeSys();
	public final HopperSys m_hopper = new HopperSys();
	public final ShooterSys m_shooter= new ShooterSys();
	public final KickerSys m_kicker= new KickerSys();
	public final TurretSys m_turret= new TurretSys();
	public final ShroudSys m_shroud = new ShroudSys();
	public final PhotonVisionSys m_photon = new PhotonVisionSys();
	public final VisionAimCmd vison = new VisionAimCmd(m_turret, m_shroud, m_photon);
	private final OI m_oi= new OI();
	private final Compressor m_compressor= new Compressor(PneumaticsModuleType.CTREPCM);
	VisionAimCmd vc = new VisionAimCmd(m_turret, m_shroud, m_photon);
	public SerialPort arduino;
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		//m_compressor.setClosedLoopControl(true); subject to change
		 m_climber.setDefaultCommand(
		 new StartEndCommand(() -> m_climber.setClimber(m_oi.getAxis(1,
		 Constants.Axes.RIGHT_STICK_Y)),
		 () -> m_climber.setClimber(0), m_climber));
		 m_climber.setPistons(DoubleSolenoid.Value.kReverse);
		
		m_drive.setDefaultCommand(new ManualDriveCmd(m_drive, m_oi));
		m_intake.setDefaultCommand(new StartEndCommand(() -> {
			if (m_oi.getAxis(1, Constants.Axes.RIGHT_TRIGGER) > 0) {
				m_hopper.setHopper(-0.2);
				m_intake.setIntake(0.6);
			} else if (m_oi.getAxis(1, Constants.Axes.LEFT_TRIGGER) > 0) {
				m_hopper.setHopper(-0.2);
				m_intake.setIntake(-0.6);
				
			} else {
				m_hopper.setHopper(0);
				m_intake.setIntake(0);
			}
		}, () -> {
			m_hopper.setHopper(0);
			m_intake.setIntake(0);
		}, m_intake, m_hopper));

		m_turret.setDefaultCommand(
				new StartEndCommand(() -> m_turret.setTurret(m_oi.getAxis(1, Constants.Axes.LEFT_STICK_X) * 0.4),
						() -> m_turret.setTurret(0), m_turret));

		// m_shroud.setDefaultCommand(
		// new ExecuteEndCommand(() -> m_shroud.setShroud(m_oi.getAxis(1,
		// Constants.Axes.RIGHT_STICK_Y) * 0.3),
		// () -> m_shroud.setShroud(0), m_shroud));
		configureButtonBindings();
		
		
	}
	
	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		//auto aim
		m_oi.getButton(1, Buttons.LEFT_BUMPER).whileHeld(vc);
		//resetShroud pos
		m_oi.getButton(1, Buttons.B_BUTTON).whileHeld(new StartEndCommand(()->m_shroud.shroudReset(0.5),()->m_shroud.shroudReset(0),m_shroud));
		m_oi.getButton(1, Buttons.B_BUTTON).whileHeld(new InstantCommand(()->m_turret.resetTurret(), m_turret));
		//m_oi.getButton(1, Buttons.LEFT_BUMPER).whenReleased(new InstantCommand(()->m_shroud.setShroud(0)), true	);
		// Auto Shoot balls
		m_oi.getButton(1, Buttons.RIGHT_BUMPER).whileHeld(new SpoolShooterCmd(m_shooter, m_kicker, Constants.TEMPSPEED ));
		m_oi.getButton(1, Buttons.RIGHT_BUMPER).whileHeld(new PushBallsCmd(m_hopper, m_intake, m_shooter));

		m_oi.getButton(1, Buttons.X_BUTTON).whileHeld(new StartEndCommand(()->m_intake.setIntake(0.4), ()->m_intake.setIntake(0), m_intake));
		m_oi.getButton(1, Buttons.X_BUTTON).whileHeld(new StartEndCommand(()->m_hopper.setHopper(0.7), ()->m_hopper.setHopper(0), m_hopper));

	
		// Toggle Shroud Presets
		//m_oi.getPovButton(1, 270).whenPressed(new InstantCommand(() -> m_shroud.setDesiredPosition(goUp()), m_shroud));
		//m_oi.getPovButton(1, 90).whenPressed(new InstantCommand(() -> m_shroud.setDesiredPosition(goDown()), m_shroud));

		m_oi.getPovButton(1, 270).whileHeld(new StartEndCommand(()->m_shroud.shroudReset(-0.7),()->m_shroud.shroudReset(0), m_shroud));
		m_oi.getPovButton(1, 90).whileHeld(new StartEndCommand(()->m_shroud.shroudReset(0.7),()->m_shroud.shroudReset(0), m_shroud));

		/*SmartDashboard.putString("DB/String 1", "RightPOV: " + m_oi.getPovButton(1, 90).get());
		SmartDashboard.putString("DB/String 2", "LeftPOV: " + m_oi.getPovButton(1, 270).get());
		SmartDashboard.putString("DB/String 3", "UpPOV:" + m_oi.getPovButton(1, 0).get());
		SmartDashboard.putString("DB/String 4", "DownPOV:" + m_oi.getPovButton(1, 180).get());*/
		// Bring intake up
		m_oi.getPovButton(1, 0)
				.whileHeld(new StartEndCommand(() -> m_intake.setPivot(0.7), () -> m_intake.setPivot(0), m_intake));

		
		//m_oi.getButton(1, Buttons.RIGHT_BUMPER).whileHeld(new InstantCommand(()->m_kicker.setKicker(0.5), m_kicker));
		//m_oi.getButton(1, Buttons.RIGHT_BUMPER).whileHeld((m_shooter.isAtSpeed())?new InstantCommand(() -> m_hopper.setHopper(0.5), m_hopper):new InstantCommand(() -> m_hopper.setHopper(0), m_hopper));
		//m_oi.getButton(1, Buttons.RIGHT_BUMPER).whileHeld((m_shooter.isAtSpeed())?new InstantCommand(() -> m_intake.setIntake(0.5), m_intake):new InstantCommand(() -> m_intake.setIntake(0), m_intake));	
		// Bring intake down
		m_oi.getPovButton(1, 180)
				.whileHeld(new StartEndCommand(() -> m_intake.setPivot(-0.5), () -> m_intake.setPivot(0), m_intake));

		// Extend the climber pistons
		m_oi.getButton(1, Buttons.Y_BUTTON)
				.whileHeld(new InstantCommand(() -> m_climber.setPistons(DoubleSolenoid.Value.kForward), m_climber));
		m_oi.getButton(1, Buttons.A_BUTTON)
				.whileHeld(new InstantCommand(() -> m_climber.setPistons(DoubleSolenoid.Value.kReverse), m_climber));

		// Move kicker wheel back to clear ball and then spool the shooter - X_Button changed to B_Button
		//m_oi.getButton(1, Buttons.B_BUTTON).whileHeld(new SpoolShooterCmd(m_shooter, m_kicker, 10000.0));
		// This command shall not be used anymore. It creates problems with the battery, kicker gearbox and motor.
		//m_oi.getButton(1, Buttons.B_BUTTON)
		//		.whileHeld(new ExecuteEndCommand(() -> m_kicker.setKicker(-0.5), () -> m_kicker.setKicker(0), m_kicker)
		//				.withTimeout(0.1).andThen(new SpoolShooterCmd(m_shooter, m_kicker, 4300)));
		//m_oi.getButton(1, Buttons.B_BUTTON).whileHeld(new InstantCommand(() -> m_hopper.setHopper(-0.5), m_hopper));
		//m_oi.getButton(1, Buttons.B_BUTTON).whileHeld(new InstantCommand(() -> m_intake.setIntake(-0.5), m_intake));

		// Use the kicker to push the balls in
		
	}
	public void shooterInfo(){
		SmartDashboard.putBoolean("UP TO SPEED", m_shooter.isAtSpeed());
		SmartDashboard.putNumber("SPEED OF SHOOTER", m_shooter.getSpeed());
	}
	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return new ParallelCommandGroup(
			new StartEndCommand(() -> m_drive.arcadeDrive(-0.5, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(1.7),
			new StartEndCommand(()->m_turret.setTurret(0.5), ()->m_turret.setTurret(0), m_turret).withTimeout(0.7), 
			new StartEndCommand(()->m_intake.setPivot(-0.7), ()->m_intake.setPivot(0), m_intake).withTimeout(0.5)
			/*new ExecuteEndCommand(()->m_shroud.setShroud(0.3), ()->m_shroud.setShroud(0), m_shroud).withTimeout(0.76)*/)
			.andThen(new SpoolShooterCmd(m_shooter, m_kicker, Constants.TEMPSPEED-100).withTimeout(2))
	.andThen(
		new ParallelCommandGroup(new SpoolShooterCmd(m_shooter, m_kicker, Constants.TEMPSPEED-100),
			new StartEndCommand(()->m_intake.setIntake(0.4),()->m_intake.setIntake(0),m_intake),
			new StartEndCommand(()->m_hopper.setHopper(0.7), ()->m_hopper.setHopper(0),m_hopper).withTimeout(3)))
	.andThen(
		new SpoolShooterCmd(m_shooter, m_kicker, 0)
	);
	}
	public Command getAutonomousCommand2() {
		return new ParallelCommandGroup(
			new StartEndCommand(() -> m_drive.arcadeDrive(-0.5, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(1.5),
			new StartEndCommand(()->m_turret.setTurret(0.5), ()->m_turret.setTurret(0), m_turret).withTimeout(0.7), 
			new StartEndCommand(()->m_intake.setPivot(-0.7), ()->m_intake.setPivot(0), m_intake).withTimeout(0.5)
			/*new ExecuteEndCommand(()->m_shroud.setShroud(0.3), ()->m_shroud.setShroud(0), m_shroud).withTimeout(0.76)*/)
			.andThen(new SpoolShooterCmd(m_shooter, m_kicker, Constants.TEMPSPEED).withTimeout(2))
	.andThen(
		new ParallelCommandGroup(
			new SpoolShooterCmd(m_shooter, m_kicker, Constants.TEMPSPEED),
			new StartEndCommand(()->m_intake.setIntake(0.4),()->m_intake.setIntake(0),m_intake),
			new StartEndCommand(()->m_hopper.setHopper(0.7), ()->m_hopper.setHopper(0),m_hopper).withTimeout(3)))
	.andThen(
		new SpoolShooterCmd(m_shooter, m_kicker, 0)
	);
	}

	public Command PathAuton(){
		var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
           Constants.DRIVETRAIN_FEED_FORWARD,
            Constants.kDriveKinematics,
			10);
		TrajectoryConfig config =
			new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
								 Constants.KMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(Constants.kDriveKinematics)
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint);
		
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
					// Pass through these two interior waypoints, making an 's' curve path
				List.of(
					/*new Translation2d(1, 1.25),
					new Translation2d(2, 1.25), 
					new Translation2d(4, 1.25)*/
					/*new Translation2d(1,0),
					new Translation2d(2,0)*/

					),
					// End 3 meters straight ahead of where we started, facing forward
					new Pose2d(1,0, Rotation2d.fromDegrees(90)),
					// Pass config 	
					config
				);

		RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_drive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.KRamseteZeta),
        Constants.DRIVETRAIN_FEED_FORWARD,
        Constants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drive::tankDriveVolts,
        m_drive
	);
	m_drive.resetOdometry(exampleTrajectory.getInitialPose());
	return null;
	/*new ParallelCommandGroup(new ExecuteEndCommand(() -> m_drive.arcadeDrive(-0.5, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(1),
	new ExecuteEndCommand(()->m_turret.setTurret(0.5), ()->m_turret.setTurret(0), m_turret).withTimeout(0.7), 
	new ExecuteEndCommand(()->m_intake.setPivot(-0.7), ()->m_intake.setPivot(0), m_intake).withTimeout(0.5))
	.andThen(new ParallelCommandGroup(
		new VisionAimCmd(m_turret, m_shroud, m_photon),
		new SpoolShooterCmd(m_shooter, m_kicker, Constants.TEMPSPEED),
		new ExecuteEndCommand(()->m_intake.setIntake(0.4),()->m_intake.setIntake(0),m_intake),
		new ExecuteEndCommand(()->m_hopper.setHopper(0.7), ()->m_hopper.setHopper(0),m_hopper).withTimeout(3))
	).andThen(ramseteCommand);*/
	}
	/*public Trajectory rotateToTrench(){
			
	}
	public Trajectory pickUpBallRotate(){

	}
	public Trajectory TrenchtoShootingPos(){

	}
	/*public Command getAutoomousCommandPath(){
		 
	
	}*/
	public void setDriveNeutralMode(NeutralMode mode) {
		m_drive.setNeutralMode(mode);
	}

	public ShroudSys getShroud()
	{
		return this.m_shroud;
	}

	/*public void resetDesiredPostition()
	{
		this.shroudPos = 0;
		m_shroud.setDesiredPosition(shroudPos);
	}*/
}
