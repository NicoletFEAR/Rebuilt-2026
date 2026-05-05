package frc.robot.io.motor;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.util.CANId;

public class MotorIONone extends MotorIO {
    public MotorIONone(CANId id, MotorType type) {
        super(id, type);
    }
}
