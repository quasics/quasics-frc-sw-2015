package frc.robot.utils;

public enum RobotSettings {
    /*
     * Supported values
     */
    Sally,
    Margaret,
    Mae,
    Simulator("photonvision", 9, 40),
    Xrp(null, 9, 40),
    Romi(null, 9, 40);

    // Vision subsystem data
    public final String cameraName;

    // Lighting subsystem data
    private static final int DEFAULT_LIGHTING_PWM_PORT = 9;
    private static final int DEFAULT_NUM_LIGHTS = 9;
    public final int lightingPwmPort;
    public final int numLights;

    private RobotSettings() {
        this("photonvision", DEFAULT_LIGHTING_PWM_PORT, DEFAULT_NUM_LIGHTS);
    }

    private RobotSettings(String cameraName, int lightingPort, int numLights) {
        this.cameraName = cameraName;
        this.numLights = numLights;
        this.lightingPwmPort = lightingPort;
    }
}
