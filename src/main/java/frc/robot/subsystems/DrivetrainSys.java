/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.*;

public class DrivetrainSys extends SubsystemBase {
	private final WPI_VictorSPX frontLeft, frontRight, backLeft, backRight;
	private final AHRS navx;
	private final DifferentialDriveOdometry odometry;
	private final DifferentialDrive drive;
	public final Encoder m_RightEncoder = new Encoder(0,1, true);
	public final Encoder m_LeftEncoder = new Encoder(2,3);
	private static final double diameter = 0.1524;//6 inches

public DrivetrainSys(){
	m_LeftEncoder.setDistancePerPulse(diameter * Math.PI / 2048.0);
	m_RightEncoder.setDistancePerPulse(diameter * Math.PI / 2048.0);

		this.navx = new AHRS(Port.kMXP);

		this.frontLeft = new WPI_VictorSPX(Constants.FRONT_LEFT_DRIVE_CAN);
		frontLeft.configFactoryDefault();
		frontLeft.setNeutralMode(NeutralMode.Brake);

		this.frontRight = new WPI_VictorSPX(Constants.FRONT_RIGHT_DRIVE_CAN);
		frontRight.configFactoryDefault();
		frontRight.setNeutralMode(NeutralMode.Brake);

		this.backLeft = new WPI_VictorSPX(Constants.BACK_LEFT_DRIVE_CAN);
		backLeft.configFactoryDefault();
		backLeft.setNeutralMode(NeutralMode.Brake);
		backLeft.follow(frontLeft);

		this.backRight = new WPI_VictorSPX(Constants.BACK_RIGHT_DRIVE_CAN);
		backRight.configFactoryDefault();
		backRight.setNeutralMode(NeutralMode.Brake);
		backRight.follow(frontRight);

		this.odometry = new DifferentialDriveOdometry(navx.getRotation2d());

		this.drive = new DifferentialDrive(frontLeft, frontRight);
		drive.setSafetyEnabled(false);
	}

	public void setNeutralMode(NeutralMode mode) {
		frontLeft.setNeutralMode(mode);
		backLeft.setNeutralMode(mode);
		frontRight.setNeutralMode(mode);
		backRight.setNeutralMode(mode);
	}

	public Pose2d getCurrentPose() {
		return odometry.getPoseMeters();
	}

	public void arcadeDrive(double speed, double rotation) {
		drive.arcadeDrive(speed, rotation);
	}
	public void setMotorVolts(double l, double r){
		frontLeft.setVoltage(l);
		frontRight.setVoltage(r);
	}

	public void stop() {
		drive.stopMotor();
	}

	public double getYaw() {
		return -navx.getYaw();
	}
	public Pose2d getPose(){
		return odometry.getPoseMeters();
	}
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_LeftEncoder.getRate(), m_RightEncoder.getRate());
	}
	public void resetEncoders(){
		m_LeftEncoder.reset();
		m_RightEncoder.reset();
	}
	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		odometry.resetPosition(pose, navx.getRotation2d());
	  }
	@Override
	public void periodic() {
		odometry.update(navx.getRotation2d(),m_LeftEncoder.getDistance(), m_RightEncoder.getDistance());
	}
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		frontLeft.setVoltage(leftVolts);
		frontRight.setVoltage(-rightVolts);
		drive.feed();
	}
	public double getAverageEncoderDistance() {
		return (m_LeftEncoder.getDistance() + m_RightEncoder.getDistance()) / 2.0;
	}

	public double getLeftEncoder(){
		return m_LeftEncoder.getDistance();
	}
	public double getRightEncoder(){
		return m_RightEncoder.getDistance();
	}
	public void setMaxOutput(double maxOutput) {
		drive.setMaxOutput(maxOutput);
	}
	public void zeroHeading() {
		navx.reset();
	}
	public double getHeading() {
		return navx.getRotation2d().getDegrees();
	}
	public double getTurnRate() {
		return -navx.getRate();
	}
}
