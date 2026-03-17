package frc.robot.subsystems.controller;

import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.universal.UniversalRequestor;

public class ControllerRequestor extends UniversalRequestor<ControllerState, ControllerName, ControllerIdentity> {
    public ControllerRequestor(ControllerName name) {
        super(name);
    }

    @Override
    public ControllerState requestHades(HadesState fullState) {
        return fullState.Controller;
    }

    @Override
    public ControllerState requestKitbot(KitbotState fullState) {
        return fullState.Controller;
    }

    @Override
    public ControllerState requestTusk(TuskState fullState) {
        return switch (m_name) {
            case DRIVER -> fullState.DriverController;
            case OPERATOR -> fullState.OperatorController;
        };
    }
}
