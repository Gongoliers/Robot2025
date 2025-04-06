// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.CAN;

public class climber extends SubsystemBase {
  private final TalonFX leader;
  private final TalonFX follower;

  //Follower followerRequest

  /** Creates a new climber. */
  public climber(CAN leaderCAN,
      CAN followerCAN) {
    // create hardware
    leader = new TalonFX(leaderCAN.id(), leaderCAN.bus());
    TalonFXConfiguration leadConfig = new TalonFXConfiguration();

     leadConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
     leadConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
     leadConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
     leadConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -343;
    leadConfig.CurrentLimits.StatorCurrentLimit = 120;
    leadConfig.CurrentLimits.StatorCurrentLimitEnable =true;
    leadConfig.CurrentLimits.SupplyCurrentLimit = 80;
    leadConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leader.getConfigurator().apply(leadConfig);

    follower = new TalonFX(followerCAN.id(), followerCAN.bus());

    //follower.setControl(new Follower(leaderCAN.id(), true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void percentOutput( double percent){
    leader.set(percent);
  }
}
