// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.math.Math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SwerveDriveTrain extends SubsystemBase {
  /** Creates a new SwerveDriveTrain. */
  private HighSwerveModule frontLeft, frontRight, backLeft, backRight;
  ArrayList<HighSwerveModule> modules;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private boolean isFieldOriented = false;

  // TODO: cambiar isOnCompetitiveField
  private boolean isOnCompetitiveField = false;
  private PIDController distancePIDController;

  private PIDController visionTurnController, visionSpeedController, visionStrafeController;
  private PIDController visionPoseXController, visionPoseYController, visionPoseTurnController;

  private Field2d field = new Field2d();

  private SlewRateLimiter speedLimiter, strafeLimiter, turnLimiter;
  public boolean cleanUpMode = false;
  private double targetMeters;

  private double visionYaw, visionTargetYaw, visionArea, visionTargetArea, visionAngle, visionTargetAngle;

  private SysIdRoutine drivesysIdRoutine;

  MutVoltage flVoltage = Volts.mutable(0);
  MutVoltage frVoltage = Volts.mutable(0);
  MutVoltage blVoltage = Volts.mutable(0);
  MutVoltage brVoltage = Volts.mutable(0);

  MutDistance flDistance = Meters.mutable(0);
  MutDistance frDistance = Meters.mutable(0);
  MutDistance blDistance = Meters.mutable(0);
  MutDistance brDistance = Meters.mutable(0);

  MutLinearVelocity flVelocity = MetersPerSecond.mutable(0);
  MutLinearVelocity frVelocity = MetersPerSecond.mutable(0);
  MutLinearVelocity blVelocity = MetersPerSecond.mutable(0);
  MutLinearVelocity brVelocity = MetersPerSecond.mutable(0);

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

    visionSpeedController = new PIDController(HighAltitudeConstants.VISION_SPEED_kP,
        HighAltitudeConstants.VISION_SPEED_kI, HighAltitudeConstants.VISION_SPEED_kD);

    visionStrafeController = new PIDController(HighAltitudeConstants.VISION_STRAFE_kP,
        HighAltitudeConstants.VISION_STRAFE_kI, HighAltitudeConstants.VISION_STRAFE_kD);

    visionTurnController = new PIDController(HighAltitudeConstants.VISION_TURN_kP,
        HighAltitudeConstants.VISION_TURN_kI, HighAltitudeConstants.VISION_TURN_kD);
    visionTurnController.enableContinuousInput(-180, 180);

    visionPoseXController = new PIDController(HighAltitudeConstants.VISION_POSE_kP,
        HighAltitudeConstants.VISION_POSE_kI, HighAltitudeConstants.VISION_POSE_kD);

    visionPoseYController = new PIDController(HighAltitudeConstants.VISION_POSE_kP,
        HighAltitudeConstants.VISION_POSE_kI, HighAltitudeConstants.VISION_POSE_kD);

    visionPoseTurnController = new PIDController(HighAltitudeConstants.VISION_POSE_TURN_kP,
        HighAltitudeConstants.VISION_POSE_TURN_kI, HighAltitudeConstants.VISION_POSE_TURN_kD);

    visionPoseTurnController.enableContinuousInput(-Math.PI, Math.PI);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);

    speedLimiter = new SlewRateLimiter(HighAltitudeConstants.SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND);
    strafeLimiter = new SlewRateLimiter(HighAltitudeConstants.SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND);
    turnLimiter = new SlewRateLimiter(HighAltitudeConstants.SWERVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);

    drivesysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(this::driveSysID, this::logDriveSysID, this));

  }

  private void driveSysID(Voltage voltage) {
    frontLeft.getDriveMotor().setVoltage(voltage.magnitude());
    frontRight.getDriveMotor().setVoltage(voltage.magnitude());
    backLeft.getDriveMotor().setVoltage(voltage.magnitude());
    backRight.getDriveMotor().setVoltage(voltage.magnitude());
  }

  private void logDriveSysID(SysIdRoutineLog log) {
    brVoltage.mut_replace(backRight.getDriveMotor().getSpeed() * RobotController.getBatteryVoltage(), Volts);
    blVoltage.mut_replace(backLeft.getDriveMotor().getSpeed() * RobotController.getBatteryVoltage(), Volts);
    frVoltage.mut_replace(frontRight.getDriveMotor().getSpeed() * RobotController.getBatteryVoltage(), Volts);
    flVoltage.mut_replace(frontLeft.getDriveMotor().getSpeed() * RobotController.getBatteryVoltage(), Volts);

    brDistance.mut_replace(backRight.getDriveDistance(), Meters);
    blDistance.mut_replace(backLeft.getDriveDistance(), Meters);
    frDistance.mut_replace(frontRight.getDriveDistance(), Meters);
    flDistance.mut_replace(frontLeft.getDriveDistance(), Meters);

    brVelocity.mut_replace(backRight.getDriveVelocity(), MetersPerSecond);
    blVelocity.mut_replace(backLeft.getDriveVelocity(), MetersPerSecond);
    frVelocity.mut_replace(frontRight.getDriveVelocity(), MetersPerSecond);
    flVelocity.mut_replace(frontLeft.getDriveVelocity(), MetersPerSecond);

    log.motor("Front right").voltage(frVoltage).linearPosition(frDistance).linearVelocity(frVelocity);
    log.motor("Front left").voltage(flVoltage).linearPosition(flDistance).linearVelocity(flVelocity);
    log.motor("Back right").voltage(brVoltage).linearPosition(brDistance).linearVelocity(brVelocity);
    log.motor("Back left").voltage(blVoltage).linearPosition(blDistance).linearVelocity(blVelocity);
  }

  public Command driveSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return drivesysIdRoutine.quasistatic(direction);
  }

  public Command driveSysIdDynamic(SysIdRoutine.Direction direction) {
    return drivesysIdRoutine.dynamic(direction);
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
          getPoseAllianceCorrected().getRotation());
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
    speed = -maxSpeed * Math.clamp(speed / maxSpeed, -1, 1);
    setModulesStates(speed, angleTarget);

    double delta = getMetersTarget() - frontLeft.getDriveDistance();

    if (Math.abs(delta) < HighAltitudeConstants.SWERVE_DRIVE_DISTANCE_ARRIVE_OFFSET) {
      stopModules();
    }

    return Math.abs(delta) < HighAltitudeConstants.SWERVE_DRIVE_DISTANCE_ARRIVE_OFFSET;
  }

  /**
   * Moves to a target pose in a straight line.
   * 
   * @param targetPose   The target pose.
   * @param maxSpeed     The maximum linear speed in m/s.
   * @param maxTurnSpeed The maximum angular velocity in rad/s
   * @return True if considered on target.
   */
  public boolean AlignWithTargetPose(Pose2d targetPose, double maxSpeed, double maxTurnSpeed) {
    var diff = targetPose.minus(getPose());
    double distance = diff.getTranslation().getNorm();
    double deltaAngle = diff.getRotation().getDegrees();

    double speedX = visionPoseXController.calculate(getPose().getX(), targetPose.getX());
    double speedY = visionPoseYController.calculate(getPose().getY(), targetPose.getY());

    double turnSpeed = visionPoseTurnController.calculate(getPose().getRotation().getRadians(),
        targetPose.getRotation().getRadians());

    speedX = Math.clamp(speedX, -maxSpeed, maxSpeed);
    speedY = Math.clamp(speedY, -maxSpeed, maxSpeed);
    turnSpeed = Math.clamp(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

    var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, turnSpeed,
        getPose().getRotation());
    driveSpeed(speeds);

    return distance < HighAltitudeConstants.VISION_POSE_ARRIVE_OFFSET &&
        Math.abs(deltaAngle) < HighAltitudeConstants.VISION_POSE_TURN_ARRIVE_OFFSET;
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
    for (var pos : Robot.getRobotContainer().getVision().getEstimatedRobotPoses())
      swerveDrivePoseEstimator.addVisionMeasurement(pos.estimatedPose.toPose2d(),
          pos.timestampSeconds);
  }

  public Pose2d getPoseAllianceCorrected() {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    var currentPos = getPose();
    if (alliance == DriverStation.Alliance.Red)
      return new Pose2d(currentPos.getX(), currentPos.getY(), currentPos.getRotation().plus(new Rotation2d(Math.PI)));
    else
      return currentPos;
  }

  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public Pose2d getPoseInverted() {
    var currentPos = getPose();
    return new Pose2d(currentPos.getX(), currentPos.getY(), currentPos.getRotation().plus(new Rotation2d(Math.PI)));
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

  public void setModulesDriveBrakeMode(boolean doBrake) {
    for (HighSwerveModule module : modules) {
      module.setDriveBrakeMode(doBrake);
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
        frontRight.getState(),
        backLeft.getState(),
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

    double turnPower = visionTurnController.calculate(getPose().getRotation().getDegrees(), angle);
    turnPower = Math.clamp(turnPower, -maxTurnPower, maxTurnPower);

    double deltaAngle = Math.deltaAngle(getPose().getRotation().getDegrees(), angle);
    boolean turnOnTarget = Math.abs(deltaAngle) < HighAltitudeConstants.VISION_TURN_ARRIVE_OFFSET;

    double speedPower = visionSpeedController.calculate(area, targetArea);
    speedPower = Math.clamp(speedPower, -maxSpeedPower, maxSpeedPower);

    double deltaArea = targetArea - area;
    boolean speedOnTarget = Math.abs(deltaArea) < HighAltitudeConstants.VISION_SPEED_ARRIVE_OFFSET;

    double strafePower = visionStrafeController.calculate(yaw, targetYaw);
    strafePower = Math.clamp(strafePower, -maxStrafePower, maxStrafePower);

    double deltaYaw = targetYaw - yaw;
    boolean strafeOnTarget = Math.abs(deltaYaw) < HighAltitudeConstants.VISION_STRAFE_ARRIVE_OFFSET;

    defaultDrive(-speedPower, -strafePower, -turnPower);

    visionAngle = getPose().getRotation().getDegrees();
    visionTargetAngle = angle;
    visionYaw = yaw;
    visionTargetYaw = targetYaw;
    visionArea = area;
    visionTargetArea = targetArea;

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
    updateOdometryWithVision();
    putOdometry();
  }

  public void setBrakeModeAllMotors(boolean brake) {
    for (var module : modules)
      module.setBrakeModeAllMotors(brake);
  }

  public void putTargetAlignTuningValues() {
    SmartDashboard.putNumber("visionAngle", visionAngle);
    SmartDashboard.putNumber("visionTargetAngle", visionTargetAngle);
    SmartDashboard.putNumber("visionYaw", visionYaw);
    SmartDashboard.putNumber("visionTargetYaw", visionTargetYaw);
    SmartDashboard.putNumber("visionArea", visionArea);
    SmartDashboard.putNumber("visionTargetArea", visionTargetArea);
  }

  public void putModuleTuningValues() {

    frontLeft.putProcessedValues("FL");
    frontRight.putProcessedValues("FR");
    backRight.putProcessedValues("BR");
    backLeft.putProcessedValues("BL");

    frontLeft.putEncoderValuesInvertedApplied("FL");
    frontRight.putEncoderValuesInvertedApplied("FR");
    backLeft.putEncoderValuesInvertedApplied("BL");
    backRight.putEncoderValuesInvertedApplied("BR");

    frontLeft.putControlTunningValues("FL");
    frontRight.putControlTunningValues("FR");
    backLeft.putControlTunningValues("BL");
    backRight.putControlTunningValues("BR");
  }

  public void putOdometry() {
    SmartDashboard.putBoolean("IsFieldOriented?", getIsFieldOriented());

    SmartDashboard.putNumber("GetXPose", getPose().getX());
    SmartDashboard.putNumber("GetYPose", getPose().getY());

    SmartDashboard.putNumber("GetDegrees", getPose().getRotation().getDegrees());

    SmartDashboard.putNumber("GetMetersTarget", getMetersTarget());
  }
}