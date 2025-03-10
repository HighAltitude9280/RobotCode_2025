// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.math.Math;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveTrain extends SubsystemBase {
  /** Creates a new SwerveDriveTrain. */
  private HighSwerveModule frontLeft, frontRight, backLeft, backRight;
  ArrayList<HighSwerveModule> modules;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  // TODO: cambiar isFieldOriented
  private boolean isFieldOriented = false;
  private boolean isOnCompetitiveField = false;
  private PIDController distancePIDController;

  private Field2d field = new Field2d();

  private SlewRateLimiter speedLimiter, strafeLimiter, turnLimiter;
  public boolean cleanUpMode = false;
  private double targetMeters;

  public SwerveDriveTrain() {
    frontLeft = new HighSwerveModule(
        RobotMap.SWERVE_FRONT_LEFT_DRIVE_MOTOR_PORT,
        RobotMap.SWERVE_FRONT_LEFT_DRIVE_MOTOR_TYPE,
        RobotMap.SWERVE_FRONT_LEFT_DRIVE_MOTOR_INVERTED,
        RobotMap.SWERVE_FRONT_LEFT_DRIVE_ENCODER_INVERTED,
        RobotMap.SWERVE_FRONT_LEFT_DIRECTION_MOTOR_PORT,
        RobotMap.SWERVE_FRONT_LEFT_DIRECTION_MOTOR_TYPE,
        RobotMap.SWERVE_FRONT_LEFT_DIRECTION_MOTOR_INVERTED,
        RobotMap.SWERVE_FRONT_LEFT_DIRECTION_ENCODER_INVERTED,
        RobotMap.SWERVE_FRONT_LEFT_ENCODED_TALON_PORT,
        RobotMap.SWERVE_FRONT_LEFT_DIRECTION_ENCODER_OFFSET_PULSES,
        RobotMap.SWERVE_FRONT_LEFT_ENCODED_TALON_INVERTED);
    frontRight = new HighSwerveModule(
        RobotMap.SWERVE_FRONT_RIGHT_DRIVE_MOTOR_PORT,
        RobotMap.SWERVE_FRONT_RIGHT_DRIVE_MOTOR_TYPE,
        RobotMap.SWERVE_FRONT_RIGHT_DRIVE_MOTOR_INVERTED,
        RobotMap.SWERVE_FRONT_RIGHT_DRIVE_ENCODER_INVERTED,
        RobotMap.SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_PORT,
        RobotMap.SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_TYPE,
        RobotMap.SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_INVERTED,
        RobotMap.SWERVE_FRONT_RIGHT_DIRECTION_ENCODER_INVERTED,
        RobotMap.SWERVE_FRONT_RIGHT_ENCODED_TALON_PORT,
        RobotMap.SWERVE_FRONT_RIGHT_DIRECTION_ENCODER_OFFSET_PULSES,
        RobotMap.SWERVE_FRONT_RIGHT_ENCODED_TALON_INVERTED);
    backLeft = new HighSwerveModule(
        RobotMap.SWERVE_BACK_LEFT_DRIVE_MOTOR_PORT,
        RobotMap.SWERVE_BACK_LEFT_DRIVE_MOTOR_TYPE,
        RobotMap.SWERVE_BACK_LEFT_DRIVE_MOTOR_INVERTED,
        RobotMap.SWERVE_BACK_LEFT_DRIVE_ENCODER_INVERTED,
        RobotMap.SWERVE_BACK_LEFT_DIRECTION_MOTOR_PORT,
        RobotMap.SWERVE_BACK_LEFT_DIRECTION_MOTOR_TYPE,
        RobotMap.SWERVE_BACK_LEFT_DIRECTION_MOTOR_INVERTED,
        RobotMap.SWERVE_BACK_LEFT_DIRECTION_ENCODER_INVERTED,
        RobotMap.SWERVE_BACK_LEFT_ENCODED_TALON_PORT,
        RobotMap.SWERVE_BACK_LEFT_DIRECTION_ENCODER_OFFSET_PULSES,
        RobotMap.SWERVE_BACK_LEFT_ENCODED_TALON_INVERTED);
    backRight = new HighSwerveModule(
        RobotMap.SWERVE_BACK_RIGHT_DRIVE_MOTOR_PORT,
        RobotMap.SWERVE_BACK_RIGHT_DRIVE_MOTOR_TYPE,
        RobotMap.SWERVE_BACK_RIGHT_DRIVE_MOTOR_INVERTED,
        RobotMap.SWERVE_BACK_RIGHT_DRIVE_ENCODER_INVERTED,
        RobotMap.SWERVE_BACK_RIGHT_DIRECTION_MOTOR_PORT,
        RobotMap.SWERVE_BACK_RIGHT_DIRECTION_MOTOR_TYPE,
        RobotMap.SWERVE_BACK_RIGHT_DIRECTION_MOTOR_INVERTED,
        RobotMap.SWERVE_BACK_RIGHT_DIRECTION_ENCODER_INVERTED,
        RobotMap.SWERVE_BACK_RIGHT_ENCODED_TALON_PORT,
        RobotMap.SWERVE_BACK_RIGHT_DIRECTION_ENCODER_OFFSET_PULSES,
        RobotMap.SWERVE_BACK_RIGHT_ENCODED_TALON_INVERTED);

    modules = new ArrayList<HighSwerveModule>();
    modules.add(frontLeft);
    modules.add(frontRight);
    modules.add(backLeft);
    modules.add(backRight);

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(HighAltitudeConstants.SWERVE_KINEMATICS, new Rotation2d(0),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, new Pose2d(0.0, 0.0, new Rotation2d(0)));

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::driveSpeed,
        HighAltitudeConstants.pathFollowerConfig,
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    distancePIDController = new PIDController(HighAltitudeConstants.SWERVE_DISTANCE_kP, 0,
        HighAltitudeConstants.SWERVE_DISTANCE_kD);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);

    speedLimiter = new SlewRateLimiter(HighAltitudeConstants.SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND);
    strafeLimiter = new SlewRateLimiter(HighAltitudeConstants.SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND);
    turnLimiter = new SlewRateLimiter(HighAltitudeConstants.SWERVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);

  }

  // By default, the Navx reports its angle as increasing when turning to its
  // right, but many wpilib functions consider the angle as increasing when moving
  // to the left (Counter Clock-Wise, or CCW).
  // The example I saw uses the raw yaw reported by the navx and switches the
  // position of the left and right wheels in the kinematics.
  // Upon testing with simulation, it turns out both wheels and navx need to work
  // with CW-increasing angles.

  public double getHeading() {
    return Robot.getRobotContainer().getNavx().getYaw();
  }

  public double getHeadingCCWPositive() {
    return -Robot.getRobotContainer().getNavx().getYaw();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Rotation2d getRotation2dCCWPositive() {
    return Rotation2d.fromDegrees(getHeadingCCWPositive());
  }

  public void defaultDrive(double speed, double strafe, double turn) {

    speed = speedLimiter.calculate(speed);
    strafe = strafeLimiter.calculate(strafe);
    turn = turnLimiter.calculate(turn);

    // 3. Scale input to teleop max speed

    speed *= HighAltitudeConstants.SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
    strafe *= HighAltitudeConstants.SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
    turn *= HighAltitudeConstants.SWERVE_DIRECTION_MAX_ANGULAR_SPEED_RADS_PER_SECOND;

    // 4. Construct the chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (getIsFieldOriented()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speed, strafe, turn,
          getPose().getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(speed, strafe, turn);
    }
    driveSpeed(chassisSpeeds);
  }

  // 5. Set the states to the swerve modules
  public void driveSpeed(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = HighAltitudeConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }
  

  /**
   * Turns the robot until it's heading the given angle using the angle reported
   * by the odometry.
   * 
   * @param angle    The target angle to which the robot is going to turn.
   * @param maxPower Maximum speed (from 0 to 1).
   * 
   * @return True if the robot has arrived to the target.
   */
  public boolean turnToAngle(double angle, double maxPower) {
    return turnToAngle(angle, maxPower, false);
  }

  /**
   * Turns the robot until it's heading the given angle.
   * 
   * @param angle    The target angle to which the robot is going to turn.
   * @param maxPower Maximum speed (from 0 to 1).
   * @param gyro     True to turn using gyro, false to use odometry.
   * 
   * @return True if the robot has arrived to the target.
   */

  public boolean turnToAngle(double angle, double maxPower, boolean gyro) {

    double delta;

    if (gyro)
      delta = Math.deltaAngle(getHeading(), angle);
    else
      delta = Math.deltaAngle(getPose().getRotation().getDegrees(), angle);

    if (Math.abs(delta) < HighAltitudeConstants.SWERVE_TURN_ARRIVE_OFFSET) {
      stopModules();
      return true;
    }

    double power = (delta / HighAltitudeConstants.SWERVE_TURN_BRAKE_DISTANCE) * maxPower;

    defaultDrive(0, 0, Math.clamp(power, -maxPower, maxPower));
    return false;

  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, HighAltitudeConstants.SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    backLeft.setState(states[2]);
    backRight.setState(states[3]);
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModulesInXPosition() {
    frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    // System.out.println("qpd Toilet");
  }

  public void setModulesStates(double mps, double angleTargetRads) {
    frontLeft.setState(new SwerveModuleState(mps, Rotation2d.fromRadians(angleTargetRads)));
    frontRight.setState(new SwerveModuleState(mps, Rotation2d.fromRadians(angleTargetRads)));
    backLeft.setState(new SwerveModuleState(mps, Rotation2d.fromRadians(angleTargetRads)));
    backRight.setState(new SwerveModuleState(mps, Rotation2d.fromRadians(angleTargetRads)));
    // System.out.println("qpd Toilet");
  }

  public boolean turnWheels(double angleTargetRads) {
    boolean onTarget = false;

    setModulesStates(0, angleTargetRads);

    boolean FL = (Math.abs(angleTargetRads
        - (frontLeft.getAbsoluteEncoderRAD() - java.lang.Math.floor(frontLeft.getAbsoluteEncoderRAD() / Math.PI)
            * Math.PI)) <= HighAltitudeConstants.SWERVE_TURN_WHEELS_RADIANS_ARRIVE_OFFSET);

    boolean FR = (Math.abs(angleTargetRads
        - (frontRight.getAbsoluteEncoderRAD() - java.lang.Math.floor(frontRight.getAbsoluteEncoderRAD() / Math.PI)
            * Math.PI)) <= HighAltitudeConstants.SWERVE_TURN_WHEELS_RADIANS_ARRIVE_OFFSET);

    boolean BL = (Math.abs(angleTargetRads
        - (backLeft.getAbsoluteEncoderRAD() - java.lang.Math.floor(backLeft.getAbsoluteEncoderRAD() / Math.PI)
            * Math.PI)) <= HighAltitudeConstants.SWERVE_TURN_WHEELS_RADIANS_ARRIVE_OFFSET);

    boolean BR = (Math.abs(angleTargetRads
        - (backRight.getAbsoluteEncoderRAD() - java.lang.Math.floor(backRight.getAbsoluteEncoderRAD() / Math.PI)
            * Math.PI)) <= HighAltitudeConstants.SWERVE_TURN_WHEELS_RADIANS_ARRIVE_OFFSET);

    onTarget = FL && FR && BL && BR;

    return onTarget;
  }

  public void setMetersTarget(double targetMeters) {
    this.targetMeters = frontLeft.getDriveDistance() + targetMeters;
  }

  private double getMetersTarget() {
    return targetMeters;
  }

  public boolean moveMeters(double maxSpeed, double angleTarget) {
    double speed = distancePIDController.calculate(targetMeters, frontLeft.getDriveDistance());
    speed = maxSpeed * Math.clamp(speed / maxSpeed, -1, 1);
    setModulesStates(speed, angleTarget);

    double delta = getMetersTarget() - frontLeft.getDriveDistance();

    if (Math.abs(delta) < HighAltitudeConstants.SWERVE_DRIVE_DISTANCE_ARRIVE_OFFSET)
      stopModules();

    return Math.abs(delta) < HighAltitudeConstants.SWERVE_DRIVE_DISTANCE_ARRIVE_OFFSET;
  }

  // Odometry
  public void updateOdometry() {
    swerveDrivePoseEstimator.update(
        getRotation2dCCWPositive(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition() });
    field.setRobotPose(getPose());
  }

  public void updateOdometryWithVision() {
    for (var pos : Robot.getRobotContainer().getVision().getEstimatedGlobalPose()) {
      if (pos == null || pos.isEmpty())
        continue;

      var pose = pos.get();
      swerveDrivePoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(),
          pose.timestampSeconds);
    }
  }

  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    swerveDrivePoseEstimator.resetPosition(getRotation2dCCWPositive(), new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition() }, pose);
  }

  // Getters for the modules

  public HighSwerveModule getFrontLeft() {
    return frontLeft;
  }

  public HighSwerveModule getFrontRight() {
    return frontRight;
  }

  public HighSwerveModule getBackLeft() {
    return backLeft;
  }

  public HighSwerveModule getBackRight() {
    return backRight;
  }

  public ArrayList<HighSwerveModule> getModules() {
    return modules;
  }
  // Parameters getters and setters

  public boolean getIsFieldOriented() {
    return isFieldOriented;
  }

  public void toggleFieldOriented() {
    isFieldOriented = !isFieldOriented;
  }

  public void setIsFieldOriented(boolean shouldBeFieldOriented) {
    isFieldOriented = shouldBeFieldOriented;
  }

  public void setModulesBrakeMode(boolean doBrake) {
    for (HighSwerveModule module : modules) {
      module.getDriveMotor();
      module.getDirectionMotor().setBrakeMode(doBrake);
      System.out.println("BrakeMode: " + doBrake);
    }
  }

  public static Command pathfindToPose(Pose2d targetPose) {
    PathConstraints constraints = new PathConstraints(
        HighAltitudeConstants.PATHFINDING_MAX_LINEAR_SPEED,
        HighAltitudeConstants.PATHFINDING_MAX_LINEAR_ACCELERATION,
        HighAltitudeConstants.PATHFINDING_MAX_ANGULAR_SPEED,
        HighAltitudeConstants.PATHFINDING_MAX_ANGULAR_ANGULAR_ACCELERATION);

    return AutoBuilder.pathfindToPose(targetPose, constraints);
  }

  public static Command pathfindThenPath(PathPlannerPath path) {

    PathConstraints constraints = new PathConstraints(
        HighAltitudeConstants.PATHFINDING_MAX_LINEAR_SPEED,
        HighAltitudeConstants.PATHFINDING_MAX_LINEAR_ACCELERATION,
        HighAltitudeConstants.PATHFINDING_MAX_ANGULAR_SPEED,
        HighAltitudeConstants.PATHFINDING_MAX_ANGULAR_ANGULAR_ACCELERATION);

    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return HighAltitudeConstants.SWERVE_KINEMATICS.toChassisSpeeds(
        // supplier for chassisSpeed, order of motors need to be the same as the
        // consumer of ChassisSpeed
        frontLeft.getState(),
        backLeft.getState(),
        frontRight.getState(),
        backRight.getState());
  }

  // see drive constants for details
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(
        HighAltitudeConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
  }

  public boolean pointToTarget(Pose2d target, double maxPower) {
    var pose = getPose();

    if (getPose() != null) {
      double deltaX = target.getX() - pose.getX();
      double deltaY = target.getY() - pose.getY();
      double angle = java.lang.Math.signum(deltaY) * 90;

      if (deltaX != 0) {
        angle = Math.toDegrees(Math.atan(Math.abs(deltaY / deltaX)));

        if (deltaY < 0 && deltaX > 0)
          angle *= -1;
        else if (deltaY > 0 && deltaX < 0)
          angle = 180 - angle;
        else if (deltaY < 0 && deltaX < 0)
          angle = angle - 180;
      }

      angle = Math.getOppositeAngle(angle);

      return turnToAngle(angle, maxPower);
    } else {
      return false;
    }
  }

  public void driveToTarget(double yaw, double maxTurnPower, double maxDrivePower) {
    setIsFieldOriented(false);
    double turnPower = (yaw / HighAltitudeConstants.SWERVE_TURN_BRAKE_DISTANCE) * maxTurnPower;
    double speed = Math.cos(Math.toRadians(yaw)) * maxDrivePower;
    double strafe = Math.sin(Math.toRadians(yaw)) * maxDrivePower;

    defaultDrive(speed, strafe, turnPower);
  }

  /**
   * Aligns with a target whose real angle is known. Note that this method will
   * disable field oriented driving
   * 
   * @param angle          The real angle of the target in degrees.
   * @param yaw            The current yaw reported by the sensors.
   * @param area           The current area of the target reported by the sensors.
   * @param yawOffset      The target yaw.
   * @param targetArea     The target area
   * @param maxTurnPower   The maximum turning power that will be passed (in
   *                       percentage, from 0 to 1).
   * @param maxSpeedPower  The maximum power that will be passed as speed (in
   *                       percentage, from 0 to 1).
   * @param maxStrafePower The maximum power that will be passed as strafe (in
   *                       percentage, from 0 to 1).
   * 
   * @return True if it's considered onTarget
   */
  public boolean alignWithTarget(double angle, double yaw, double area, double targetYaw, double targetArea,
      double maxTurnPower, double maxSpeedPower, double maxStrafePower) {

    double deltaAngle = Math.deltaAngle(getPose().getRotation().getDegrees(), angle);
    double turnPower = deltaAngle / HighAltitudeConstants.VISION_TURN_BRAKE_DISTANCE;
    turnPower = Math.clamp(maxTurnPower * turnPower, -maxTurnPower, maxTurnPower);

    boolean turnOnTarget = Math.abs(deltaAngle) < HighAltitudeConstants.VISION_TURN_ARRIVE_OFFSET;
    turnPower = turnOnTarget ? 0 : turnPower;

    double deltaArea = targetArea - area;
    double speedPower = deltaArea / HighAltitudeConstants.VISION_SPEED_BRAKE_DISTANCE;
    speedPower = Math.clamp(speedPower * maxSpeedPower, -maxSpeedPower, maxSpeedPower);

    boolean speedOnTarget = Math.abs(deltaArea) < HighAltitudeConstants.VISION_SPEED_ARRIVE_OFFSET;
    speedPower = speedOnTarget ? 0 : speedPower;

    double deltaYaw = targetYaw - yaw;
    double strafePower = deltaYaw / HighAltitudeConstants.VISION_STRAFE_BRAKE_DISTANCE;
    strafePower = Math.clamp(strafePower * maxStrafePower, -maxStrafePower, maxStrafePower);

    boolean strafeOnTarget = Math.abs(deltaYaw) < HighAltitudeConstants.VISION_STRAFE_ARRIVE_OFFSET;
    strafePower = strafeOnTarget ? 0 : strafePower;

    defaultDrive(speedPower, strafePower, turnPower);

    System.out.println("Turn Power" + turnPower);

    return turnOnTarget && speedOnTarget && strafeOnTarget;

  }

  public boolean getIsOnCompetitiveField() {
    return isOnCompetitiveField;
  }

  public void toggleIsOnCompetitiveField() {
    isOnCompetitiveField = !isOnCompetitiveField;
  }

  @Override
  public void periodic() {
    updateOdometry();
    // updateOdometryWithVision();
    putAllInfoInSmartDashboard();

  }

  public void putAllInfoInSmartDashboard() {

    // frontLeft.putProcessedValues("FL");
    // frontRight.putProcessedValues("FR");
    // backRight.putProcessedValues("BR");
    // backLeft.putProcessedValues("BL");

    // frontLeft.putEncoderValuesInvertedApplied("FL");
    // frontRight.putEncoderValuesInvertedApplied("FR");
    // backLeft.putEncoderValuesInvertedApplied("BL");
    // backRight.putEncoderValuesInvertedApplied("BR");

    frontLeft.putControlTunningValues("FL");
    frontRight.putControlTunningValues("FR");
    backLeft.putControlTunningValues("BL");
    backRight.putControlTunningValues("BR");

    SmartDashboard.putBoolean("IsFieldOriented?", getIsFieldOriented());

    SmartDashboard.putNumber("GetXPose", getPose().getX());
    SmartDashboard.putNumber("GetYPose", getPose().getY());

    SmartDashboard.putNumber("GetDegrees", getPose().getRotation().getDegrees());

    SmartDashboard.putNumber("GetMetersTarget", getMetersTarget());
  }
}