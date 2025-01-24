package frc.robot.resources.components;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Navx {
    private double yaw;
    private double rate;
    private double acceleration;
    private AHRS ahrs;
    private double last_world_linear_accel_x;
    private double last_world_linear_accel_y;
    private double curr_world_linear_accel_x;
    private double curr_world_linear_accel_y;
    private double currentJerkX;
    private double currentJerkY;
    private double kCollisionThreshold_DeltaG = 1.5f;

    private double last_time;
    private double last_angular_velocity_pitch, last_angular_velocity_roll;
    private double angular_acceleration_pitch, angular_acceleration_roll;

    public Navx() {
        try {
            ahrs = new AHRS(NavXComType.kMXP_SPI);
            DriverStation.reportWarning("Navx running", true);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error stantiating navX-MXP:  " + ex.getMessage(), true);
        }
        last_world_linear_accel_x = 0.0f;
        last_world_linear_accel_y = 0.0f;

        last_angular_velocity_pitch = 0;
        last_angular_velocity_roll = 0;
        last_time = Timer.getFPGATimestamp();
    }

    public void run() {
        yaw = ahrs.getYaw();
        rate = ahrs.getRawMagY();
        acceleration = ahrs.getRawAccelY();

        double delta_angular_velocity_pitch = last_angular_velocity_pitch - ahrs.getRawGyroX();
        angular_acceleration_pitch = delta_angular_velocity_pitch / (Timer.getFPGATimestamp() - last_time);
        double delta_angular_velocity_roll = last_angular_velocity_roll - ahrs.getRawGyroY();
        angular_acceleration_roll = delta_angular_velocity_roll / (Timer.getFPGATimestamp() - last_time);

        last_time = Timer.getFPGATimestamp();
    }

    public double getYaw() {
        return ahrs.getYaw();
    }

    public double getPitch() {
        return ahrs.getPitch();
    }

    public double getRoll() {
        return ahrs.getRoll();
    }

    public double getGyro() {
        return yaw + 180;
    }

    public double getMagnitud() {
        return rate;
    }

    public double getAcceleration() {
        return acceleration;
    }

    /**
     * 
     * @return the rate at which the angular velocity of the x-axis (pitch) changes
     *         in degrees per square second.
     */
    public double getAngularAccelerationPitch() {
        return angular_acceleration_pitch;
    }

    /**
     * 
     * @return the rate at which the angular velocity of the x-axis (pitch) changes
     *         in degrees per square second.
     */
    public double getAngularAccelerationRoll() {
        return angular_acceleration_roll;
    }

    public void reset() {
        ahrs.reset();
    }

    public float getWorldLinearAccelY() {
        return ahrs.getWorldLinearAccelY();
    }

    private float getWorldLinearAccelX() {
        return ahrs.getWorldLinearAccelX();
    }

    public float getXVel() {
        return ahrs.getRawGyroX();
    }

    public float getYVel() {
        return ahrs.getRawGyroY();
    }

    public boolean getCollision() {
        boolean collisionDetected = false;
        curr_world_linear_accel_x = getWorldLinearAccelX();
        currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        curr_world_linear_accel_y = getWorldLinearAccelY();
        currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;
        if ((java.lang.Math.abs(currentJerkX) > kCollisionThreshold_DeltaG) ||
                (java.lang.Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
            collisionDetected = true;
            System.out.println("Collision detected");
        }
        return collisionDetected;
    }

    public double getCollisionY() {
        curr_world_linear_accel_y = getWorldLinearAccelY();
        currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;
        return currentJerkY;
    }

    public double getCollisionX() {
        curr_world_linear_accel_x = getWorldLinearAccelX();
        currentJerkY = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        return currentJerkX;
    }

}
