/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.*;

public class ShroudSys extends SubsystemBase {
	public final WPI_VictorSPX shroud;
	final PIDController pid;
	public final Encoder shroudEncoder;
	public double positionDesired=0;
	public double pitch = 0;
	public double pidOut;
	public ShroudSys() {
		this.shroud = new WPI_VictorSPX(Constants.SHROUD_CAN);
		shroud.configFactoryDefault();
		shroud.setNeutralMode(NeutralMode.Brake);
		shroud.setInverted(InvertType.InvertMotorOutput);

		//Encoder initialization
		shroudEncoder = new Encoder(4,5, true);
		shroudEncoder.setDistancePerPulse(360.0/2048);
		//shroudEncoder.reset();
		// PID Initialization
		this.pid = new PIDController(Constants.SHROUD_KP, Constants.SHROUD_KI, Constants.SHROUD_KD);
		this.pid.setSetpoint(0);
		pid.setTolerance(1);
	}
	@Override
	public void periodic() {
		pidOut = pid.calculate(pitch, 0);
		shroud.set(pidOut);
		/*if(getDegrees()>10&&getDegrees()<490){//change for new bounds
			
		}else{
			shroud.set(0);
		}
		 //uncomment for future use FOR MANUAL CONTROL OF SHROUD
		 */
		
	}
	public double getDegrees() {
		return shroudEncoder.getDistance();
	}
	//6.5 offset
	public void setPitch(double pitch){
		this.pitch=pitch;
	}
	public void setShroud(double power) {
		if(power ==0){
			pitch=0;
			pidOut = 0;
			shroud.setVoltage(0);
			pid.reset();
		}else{
			if(getDegrees()<=-100&&power>0){
				shroud.set(power);
			}else if(getDegrees()>=480&&power<0){
				shroud.set(power);
			}
		}
		
	}
	public void shroudReset(double power){
		if(getDegrees()<=-100&&power>0){
			shroud.set(power);
		}else if(getDegrees()>=480&&power<0){
			shroud.set(power);
		}
	}
	public void resetEncoder()
	{
		shroudEncoder.reset();
	}
}
