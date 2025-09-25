package frc.robot.subsystems.Intake;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// Motores
CANSparkMax indexerMotor = new CANSparkMax(10, MotorType.kBrushless);
RelativeEncoder indexerEncoder = indexerMotor.getEncoder();

// Umbrales de detección (ajusta con pruebas)
final double CURRENT_THRESHOLD = 25.0; // Amps
final double VELOCITY_THRESHOLD = 100.0; // RPM

// Dentro del periodic()
public void detectCoral() {
    double current = indexerMotor.getOutputCurrent();
    double velocity = indexerEncoder.getVelocity();

    if (current > CURRENT_THRESHOLD || Math.abs(velocity) < VELOCITY_THRESHOLD) {
        // Coral detectado
        indexerMotor.stopMotor();

        // Cambia estado de la máquina PENDIENTE
    }
}
