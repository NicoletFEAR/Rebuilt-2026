// package frc.robot.subsystems.turn;

// import frc.robot.robots.hades.HadesState;
// import frc.robot.robots.kitbot.KitbotState;
// import frc.robot.robots.tusk.TuskState;
// import frc.robot.subsystems.base.universal.UniversalRequestor;

// public class TurnRequestor extends UniversalRequestor<TurnState, TurnName, TurnIdentity> {
//     public TurnRequestor(TurnName name) {
//         super(name);
//     }

//     @Override
//     public TurnState requestHades(HadesState fullState) {
//         return switch (m_name) {
//             case FRONT_LEFT -> fullState.FrontLeftTurn;
//             case FRONT_RIGHT -> fullState.FrontRightTurn;
//             case REAR_LEFT -> fullState.RearLeftTurn;
//             case REAR_RIGHT -> fullState.RearRightTurn;
//         };
//     }

//     @Override
//     public TurnState requestKitbot(KitbotState fullState) {
//         return switch (m_name) {
//             case FRONT_LEFT -> fullState.FrontLeftTurn;
//             case FRONT_RIGHT -> fullState.FrontRightTurn;
//             case REAR_LEFT -> fullState.RearLeftTurn;
//             case REAR_RIGHT -> fullState.RearRightTurn;
//         };
//     }

//     @Override
//     public TurnState requestTusk(TuskState fullState) {
//         return switch (m_name) {
//             case FRONT_LEFT -> fullState.FrontLeftTurn;
//             case FRONT_RIGHT -> fullState.FrontRightTurn;
//             case REAR_LEFT -> fullState.RearLeftTurn;
//             case REAR_RIGHT -> fullState.RearRightTurn;
//         };
//     }
// }
