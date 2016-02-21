/**
 * Created by Camiel on 21-Feb-16.
 */
public class Metrics {

    private static float heading = 0;
    private static float elevation = 0;
    private static float airspeed = 0;
    private static float pitch = 0;
    private static float yaw = 0;
    private static float roll = 0;
    private static float throttleNE = 0;
    private static float throttleSE = 0;
    private static float throttleSW = 0;
    private static float throttleNW = 0;


    public static float getAirspeed() {
        return airspeed;
    }

    public static void setAirspeed(float airspeed) {
        Metrics.airspeed = airspeed;
    }

    public static float getHeading() {
        return heading;
    }

    public static void setHeading(float heading) {
        Metrics.heading = heading;
    }

    public static float getElevation() {
        return elevation;
    }

    public static void setElevation(float elevation) {
        Metrics.elevation = elevation;
    }

    public static float getPitch() {
        return pitch;
    }

    public static void setPitch(float pitch) {
        Metrics.pitch = pitch;
    }

    public static float getYaw() {
        return yaw;
    }

    public static void setYaw(float yaw) {
        Metrics.yaw = yaw;
    }

    public static float getRoll() {
        return roll;
    }

    public static void setRoll(float roll) {
        Metrics.roll = roll;
    }

    public static float getThrottleNE() {
        return throttleNE;
    }

    public static void setThrottleNE(float throttleNE) {
        Metrics.throttleNE = throttleNE;
    }

    public static float getThrottleSE() {
        return throttleSE;
    }

    public static void setThrottleSE(float throttleSE) {
        Metrics.throttleSE = throttleSE;
    }

    public static float getThrottleSW() {
        return throttleSW;
    }

    public static void setThrottleSW(float throttleSW) {
        Metrics.throttleSW = throttleSW;
    }

    public static float getThrottleNW() {
        return throttleNW;
    }

    public static void setThrottleNW(float throttleNW) {
        Metrics.throttleNW = throttleNW;
    }

}
