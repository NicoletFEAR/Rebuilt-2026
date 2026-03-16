package frc.robot.subsystems.controller;

import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.universal.UniversalRequestor;

public class ControllerRequestor extends UniversalRequestor<ControllerState> {
    private ControllerName m_name;

    public ControllerRequestor(ControllerName name) {
        m_name = name;
    }

    @Override
    public ControllerState requestHades(HadesState fullState) {
        return fullState.ControllerState;
    }

    @Override
    public ControllerState requestKitbot(KitbotState fullState) {
        return fullState.ControllerState;
    }

    @Override
    public ControllerState requestTusk(TuskState fullState) {
        return switch (m_name) {
            case DRIVER -> fullState.DriverControllerState;
            case OPERATOR -> fullState.OperatorControllerState;
        };
    }
}
