/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSys;
import frc.robot.subsystems.ShroudSys;
import frc.robot.subsystems.TurretSys;

import edu.wpi.first.wpilibj2.command.*;
//BUTTON TO USE THIS PROGRAM IS THE LEFT BUMPER ON THE SECOND REMOTE, OR REMOTE #1
public class VisionAimCmd extends CommandBase {
	private final TurretSys turret;
	private final ShroudSys shroud; 
	private final PhotonVisionSys photon;
	public double yaw;
	public double pitch;
	public VisionAimCmd(TurretSys turret,ShroudSys shroud, PhotonVisionSys photon) {
		this.turret = turret;
		this.shroud = shroud;
		this.photon = photon;
		addRequirements(turret, shroud);
	}
	@Override
	//go to 10.46.39.11:5801 to see camera output and tune it
	public void execute() {
		
		boolean target_found = 
			1==photon.LLTable.getEntry("tv").getDouble(0);//whether a target is found
		if(target_found){
			yaw = photon.LLTable.getEntry("tx").getDouble(0);
			pitch =photon.LLTable.getEntry("ty").getDouble(0.0);
			
			turret.align(yaw);
			shroud.pitch = pitch;
		}

	}

	@Override
	public void end(boolean interrupted) {
		shroud.pitch = 0;
		shroud.shroud.setVoltage(0);
		turret.setTurret(0);
		shroud.setShroud(0);
		
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
