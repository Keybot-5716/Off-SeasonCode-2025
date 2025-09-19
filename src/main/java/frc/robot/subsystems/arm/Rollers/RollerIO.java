package frc.robot.subsystems.arm.Rollers;

import org.littletonrobotics.junction.Autolog;


public interface RollerIO {

    @Autolog
    class RollerIOInputs {
        public RollerIOData data = new RollerIOData(false, false, 0.0, 0.0, 0.0, 0.0);
    }

    record RollerIOData(

        boolean motorConnected,
        boolean intaking,
        double velocityRPS,
        double RollerappliedVoltage,
        double RollercurrentAmps,
        double RollertemperatureCelsius) {}

        public default void updateInputs(RollerIOInputs inputs) {}

        public default void setRollerSpeed(double speed) {}

        public default void stopRoller() {}

        public default void runOpenLoop(double output) {}
    }
    
}
