/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.controller.PIDController;

public class TurretSys extends SubsystemBase {
	private final WPI_TalonSRX turret;
	private final PIDController pid;
	public TurretSys() {
		this.turret = new WPI_TalonSRX(Constants.TURRET_CAN);
		turret.configFactoryDefault();
		turret.setNeutralMode(NeutralMode.Brake);
		turret.setInverted(InvertType.InvertMotorOutput);
		pid = new PIDController(Constants.TURRET_KP, Constants.TURRET_KI, Constants.TURRET_KD);
		pid.setSetpoint(0);
		pid.setTolerance(0);
	}
	public void resetTurret(){//NNED TO BE CHANGED, ANOTHER PID?
		int defPos = -3250;
		double degs = getDegrees();
		if(degs<defPos){
			setTurret(Constants.KP_ROT_TURRET*(degs/25)+Constants.CONSTANT_FORCE_TURRET);
		}else
			setTurret(Constants.KP_ROT_TURRET*(degs/25)-Constants.CONSTANT_FORCE_TURRET);
	}
	public void setTurretPos(double pos){//NEEDS TO BE CHANGED TO PID CONTROLLER
		double degs = getDegrees();
		if(degs<pos){
			setTurret(Constants.KP_ROT_TURRET*(degs/25)+Constants.CONSTANT_FORCE_TURRET);
		}else
			setTurret(Constants.KP_ROT_TURRET*(degs/25)-Constants.CONSTANT_FORCE_TURRET);
	}
	public void setTurret(double power) {
		turret.set(power);
	}
	public void setTurretV(double v){
		turret.setVoltage(v);
	}
	public double getDegrees() {
		return turret.getSelectedSensorPosition()+11250;
	}
	@Override
	public void periodic() {
		//ADD CODE HERE
		
	}
	public void align(double degOff){
		double pidOut = pid.calculate(degOff);
		turret.set(-pidOut);
	}
}
