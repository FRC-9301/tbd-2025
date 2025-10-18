// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants.IntakeConstants.CollectingVoltages;
import frc.robot.constants.Constants.IntakeConstants.EjectingVoltages;
import frc.robot.constants.Constants.IntakeConstants.HoldingVoltages;

public class Intake extends SubsystemBase {
  private final SparkMax topMotor;
  private final SparkMax vertMotorR;
  private final SparkMax vertMotorL;
  private final SparkMaxConfig topMotorConfig;
  private final SparkMaxConfig vertMotorRConfig;
  private final SparkMaxConfig vertMotorLConfig;


  public Intake() {
    topMotor = new SparkMax(27, MotorType.kBrushless);
    vertMotorR = new SparkMax(26, MotorType.kBrushless);
    vertMotorL = new SparkMax(25, MotorType.kBrushless);

    topMotorConfig = new SparkMaxConfig();
    vertMotorRConfig = new SparkMaxConfig();
    vertMotorLConfig = new SparkMaxConfig();
        
        topMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
        vertMotorRConfig
        .follow(vertMotorL, true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
        vertMotorLConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);

    topMotor.configure(topMotorConfig, null, null);
    vertMotorR.configure(vertMotorRConfig, null, null);
    vertMotorL.configure(vertMotorLConfig, null, null);
    
  }
  private Command setVoltageTop(double volts){
    return Commands.runOnce(() -> topMotor.setVoltage(volts), this);
  }

  private Command setVoltageVert(double volts){
    return Commands.runOnce(() -> vertMotorL.setVoltage(volts), this);
  }

  private Command setVoltageBoth(double volts){
    return Commands.runOnce(() -> {
      topMotor.setVoltage(volts);
      vertMotorL.setVoltage(volts);
    }, this);

  }
  public Command stopMotors(){
    return Commands.runOnce(() -> {
      topMotor.setVoltage(0);
      vertMotorL.setVoltage(0);
    }, this);
  }

  // Algae commands
  public Command algaeCollect (){;
    return setVoltageTop(CollectingVoltages.COLLECTING_TOP_ALGAE_VOLTAGE);
  }
  public Command algaeProc (){
    return setVoltageTop(EjectingVoltages.EJECTING_PROCESSOR_TOP_ALGAE_VOLTAGE);
  }
  public Command algaeHold (){
    return setVoltageTop(HoldingVoltages.HOLDING_TOP_ALGAE_VOLTAGE);
  }

  // Coral commands
  public Command coralCollectGround (){
    return setVoltageBoth(CollectingVoltages.COLLECTING_VERT_CORAL_VOLTAGE);
  }
  public Command coralCollectStation (){
    return setVoltageVert(CollectingVoltages.COLLECTING_VERT_CORAL_VOLTAGE);
  }
  public Command coralL1 (){
    return setVoltageVert(EjectingVoltages.EJECTING_VERTICAL_CORAL_VOLTAGE_L1);
  }
  public Command coralL2 (){
    return setVoltageVert(EjectingVoltages.EJECTING_VERTICAL_CORAL_VOLTAGE_L2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

