package frc.robot.utils;

/**
 * Defines bot-specific configuration data for every one of Quasics' supported
 * instances in 2024.
 */
public interface RobotSettings {
    public enum MotorConfigModel {
        NoLeader, RearMotorsLeading, FrontMotorsLeading
    }

    public enum Robot {
        /*
         * Supported values
         */
        Sally,
        Margaret,
        Mae,
        Simulator(MotorConfigModel.NoLeader, "photonvision", 9, 40),
        Xrp(MotorConfigModel.NoLeader, null, 9, 40),
        Romi(MotorConfigModel.NoLeader, null, 9, 40);

        // Drive base data
        public final MotorConfigModel motorConfigModel;

        // Vision subsystem data
        public final String cameraName;

        // Lighting subsystem data
        private static final int DEFAULT_LIGHTING_PWM_PORT = 9;
        private static final int DEFAULT_NUM_LIGHTS = 9;
        public final int lightingPwmPort;
        public final int numLights;

        private Robot() {
            this(MotorConfigModel.NoLeader, "photonvision", DEFAULT_LIGHTING_PWM_PORT, DEFAULT_NUM_LIGHTS);
        }

        private Robot(MotorConfigModel motorConfigModel, String cameraName, int lightingPort, int numLights) {
            this.motorConfigModel = motorConfigModel;
            this.cameraName = cameraName;
            this.numLights = numLights;
            this.lightingPwmPort = lightingPort;
        }
    }
}
