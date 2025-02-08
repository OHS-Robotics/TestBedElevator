// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ElevatorJogUp;
import frc.robot.commands.ElevatorJogDown;
import frc.robot.commands.ElevatorStop;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {

  ElevatorSubsystem elevator = new ElevatorSubsystem(1);
  CommandJoystick joystick = new CommandJoystick(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    ElevatorJogUp jogUpCommand = new ElevatorJogUp(elevator);
    ElevatorJogDown jogDownCommand = new ElevatorJogDown(elevator);
    ElevatorStop stopCommand = new ElevatorStop(elevator);

    jogUpCommand.addRequirements(elevator);
    jogDownCommand.addRequirements(elevator);
    stopCommand.addRequirements(elevator);

    joystick.povUp().onTrue(jogUpCommand);
    joystick.povUp().onFalse(stopCommand);
    joystick.povDown().onTrue(jogDownCommand);
    joystick.povDown().onFalse(stopCommand);
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
