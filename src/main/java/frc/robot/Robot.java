/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SpoolShooterCmd;
import frc.robot.commands.VisionAimCmd;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();
		CameraServer.startAutomaticCapture();
		//m_robotContainer.arduino.write(new byte[]{0x1},0);
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		//m_robotContainer.arduino.write(new byte[]{0x1},0);
		//System.out.println(m_robotContainer.arduino.readString());
		CommandScheduler.getInstance().run();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
		//m_robotContainer.arduino.write(new byte[]{0x1},0);
	}

	@Override
	public void disabledPeriodic() {
		//m_robotContainer.arduino.write(new byte[]{0x1},0);
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		//m_robotContainer.arduino.write(new byte[]{0x2},0);
		m_robotContainer.setDriveNeutralMode(NeutralMode.Brake);
		//m_robotContainer.m_shroud.resetEncoder();
		m_autonomousCommand = m_robotContainer.PathAuton();
		
		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		//m_robotContainer.arduino.write(new byte[]{0x2},0);
		//SmartDashboard.putNumber("Time:",time.get());
		/*VisionAimCmd vs =new VisionAimCmd(m_robotContainer.m_turret, m_robotContainer.m_shroud, m_robotContainer.m_photon);
		vs.pitch-=40;
		vs.execute();*/
	}

	@Override
	public void teleopInit() {
		//m_robotContainer.arduino.write(new byte[]{0x3},0);
		m_robotContainer.setDriveNeutralMode(NeutralMode.Brake);
		//Reset the encoders on the shroud encoder
		m_robotContainer.getShroud().resetEncoder();
		//m_robotContainer.resetDesiredPostition();
		
		m_robotContainer.m_drive.m_LeftEncoder.reset();
		m_robotContainer.m_drive.m_RightEncoder.reset();
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
	//	m_robotContainer.arduino.write(new byte[]{0x3},0);
		//System.out.println(m_robotContainer.arduino.readString());
		SmartDashboard.putNumber("Target:", m_robotContainer.m_photon.LLTable.getEntry("tv").getDouble(0.0));
		SmartDashboard.putNumber("HorizontalOffset", m_robotContainer.m_photon.LLTable.getEntry("tx").getDouble(0.0));
		SmartDashboard.putNumber("HorizontalOff", m_robotContainer.m_photon.LLTable.getEntry("tx").getDouble(0.0));
		SmartDashboard.putNumber("VerticalOffset", m_robotContainer.m_photon.LLTable.getEntry("ty").getDouble(0.0));
	
		SmartDashboard.putNumber("leftSideValue", m_robotContainer.m_drive.m_LeftEncoder.getDistance());
		SmartDashboard.putNumber("rightSideValue", m_robotContainer.m_drive.m_RightEncoder.getDistance());
		SmartDashboard.putNumber("Shroud pitch value", m_robotContainer.m_shroud.shroudEncoder.getDistance());
		SmartDashboard.putNumber("Shroud power value", m_robotContainer.m_shroud.pidOut);

		SmartDashboard.putNumber("TurretSpeed:", m_robotContainer.m_shooter.getSpeed());
		SmartDashboard.putNumber("Desired Speed:", m_robotContainer.m_shooter.speedDesired);
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
