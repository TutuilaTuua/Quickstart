package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Master Auto", group = "Auto")
public class MasterAuto extends OpMode {

    private enum Alliance { BLUE, RED }

    private enum Side { LEFT, RIGHT }

    private enum State { DRIVE_TO_SHOOT, SPINUP, FEED, RECOVER, DRIVE_TO_PICKUP, INTAKE_SWEEP, INTAKE_DWELL, PARK, DONE }

    // Hardware names
    private static final String SHOOTER_NAME = "shooter";
    private static final String INTAKE_NAME = "intake";
    private static final String FEEDER_NAME = "feeder";
    private static final String INTAKE_SERVO_NAME = "left_intake_servo";
    private static final String WEBCAM_NAME = "Webcam 1";

    // Motor directions
    private static final DcMotorSimple.Direction SHOOTER_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction INTAKE_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction FEEDER_DIR = DcMotorSimple.Direction.FORWARD;

    // Shooter velocity control (ticks per second)
    private static final double SHOOTER_TARGET_VEL = 1500;
    private static final double SHOOTER_TOLERANCE = 50;
    private static final double SHOOTER_P = 12.0;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 11.7;

    // Battery compensation
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MIN_COMP_SCALE = 0.8;
    private static final double MAX_COMP_SCALE = 1.3;
    private static final double LOW_VOLTAGE_WARN = 10.5;
    private static final double VOLTAGE_REFRESH_S = 0.5;
    private static final double VOLTAGE_DEADBAND = 0.2;

    // Mechanism powers and servo positions
    private static final double INTAKE_POWER = 1.0;
    private static final double FEEDER_POWER = 1.0;
    private static final double SERVO_STOW_POS = 0.0;
    private static final double SERVO_DEPLOY_POS = 0.5;

    // Shooting sequence timing
    private static final int SHOTS_PER_CYCLE = 3;
    private static final double FEED_PULSE_S = 0.35;
    private static final double READY_HOLD_S = 0.15;
    private static final double SPINUP_TIMEOUT_S = 3.0;
    private static final double RECOVER_TIMEOUT_S = 1.0;

    // Path timeouts and intake dwell
    private static final double PATH_TIMEOUT_S = 4.0;
    private static final double SWEEP_TIMEOUT_S = 3.5;
    private static final double INTAKE_DWELL_S = 0.5;

    // Match clock
    private static final double AUTO_LIMIT_S = 29.5;
    private static final double AUTO_TOTAL_S = 30.0;
    private static final double MIN_CYCLE_TIME_S = 12.0;

    // Field mirror line used for the red alliance
    private static final double FIELD_MIRROR_X = 70.75;

    // Pose handoff for a following TeleOp
    public static Pose lastPose = null;

    private static final class Plan {
        final Pose start, shoot, pickupStart, pickupEnd, park;

        Plan(Pose start, Pose shoot, Pose pickupStart, Pose pickupEnd, Pose park) {
            this.start = start;
            this.shoot = shoot;
            this.pickupStart = pickupStart;
            this.pickupEnd = pickupEnd;
            this.park = park;
        }
    }

    private Follower follower;
    private DcMotorEx shooter;
    private DcMotorEx intake;
    private DcMotorEx feeder;
    private Servo intakeServo;
    private VoltageSensor voltageSensor;
    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;

    private final List<String> missingDevices = new ArrayList<>();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private Alliance alliance = Alliance.BLUE;
    private Side side = Side.LEFT;
    private Plan plan;
    private State state = State.DRIVE_TO_SHOOT;
    private boolean spinupTimedOut = false;
    private int cycle = 0;
    private int shotsFired = 0;
    private double shooterVel = 0;
    private double appliedVoltage = -1;
    private double batteryVoltage = NOMINAL_VOLTAGE;
    private String motif = "UNKNOWN";
    private String visionError = "";

    @Override
    public void init() {
        // Enables bulk caching on every hub
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initializes drivetrain follower
        follower = Constants.create(hardwareMap);

        // Initializes mechanisms
        shooter = tryMotor(SHOOTER_NAME, SHOOTER_DIR, DcMotor.ZeroPowerBehavior.FLOAT, true);
        intake = tryMotor(INTAKE_NAME, INTAKE_DIR, DcMotor.ZeroPowerBehavior.BRAKE, false);
        feeder = tryMotor(FEEDER_NAME, FEEDER_DIR, DcMotor.ZeroPowerBehavior.BRAKE, false);
        intakeServo = hardwareMap.tryGet(Servo.class, INTAKE_SERVO_NAME);
        if (intakeServo == null) {
            missingDevices.add(INTAKE_SERVO_NAME);
        } else {
            intakeServo.setPosition(SERVO_STOW_POS);
        }

        // Finds the battery voltage sensor
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensor = sensor;
            break;
        }

        // Applies shooter PIDF at nominal voltage
        applyShooterPidf(1.0);

        initVision();

        plan = buildPlan(alliance, side);
        follower.setPose(plan.start);
    }

    @Override
    public void init_loop() {
        // Reads alliance and side selection from gamepad 1
        Alliance prevAlliance = alliance;
        Side prevSide = side;
        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;
        if (gamepad1.dpad_left) side = Side.LEFT;
        if (gamepad1.dpad_right) side = Side.RIGHT;

        // Rebuilds the plan only when the selection changes
        if (alliance != prevAlliance || side != prevSide) {
            plan = buildPlan(alliance, side);
            follower.setPose(plan.start);
        }

        // Reads the obelisk motif tag
        readMotif();

        telemetry.addLine("X = Blue | B = Red | DPad Left/Right = Side");
        telemetry.addData("Selected", alliance + " / " + side);
        telemetry.addData("Start pose", fmt(plan.start));
        telemetry.addData("Motif", motif);
        telemetry.addData("Battery V", "%.2f", readVoltage());
        reportWarnings();
        telemetry.update();
    }

    @Override
    public void start() {
        matchTimer.reset();

        // Frees camera bandwidth during the run
        closeVision();

        // Locks in the final start pose and deploys the intake servo
        follower.setPose(plan.start);
        if (intakeServo != null) intakeServo.setPosition(SERVO_DEPLOY_POS);

        setState(State.DRIVE_TO_SHOOT);
    }

    @Override
    public void loop() {
        // Updates localization and path following
        follower.update();
        shooterVel = (shooter != null) ? shooter.getVelocity() : 0;
        trackShooterReady();
        refreshVoltageComp();

        // Forces a stop at the match time limit
        if (state != State.DONE && matchTimer.seconds() >= AUTO_LIMIT_S) {
            setState(State.DONE);
        }

        runState();

        telemetry.addData("State", state);
        telemetry.addData("Cycle / Shots", cycle + " / " + shotsFired);
        telemetry.addData("Time left", "%.1f", AUTO_TOTAL_S - matchTimer.seconds());
        telemetry.addData("Pose", fmt(follower.pose()));
        telemetry.addData("Follower mode", follower.mode());
        telemetry.addData("Shooter vel / target", "%.0f / %.0f", shooterVel, SHOOTER_TARGET_VEL);
        telemetry.addData("Spinup timed out", spinupTimedOut);
        telemetry.addData("Battery V", "%.2f", batteryVoltage);
        reportWarnings();
        telemetry.update();
    }

    @Override
    public void stop() {
        // Saves pose and shuts everything down
        if (follower != null) lastPose = follower.pose();
        stopMechanisms();
        closeVision();
    }

    // Runs the logic for the current state
    private void runState() {
        switch (state) {
            case DRIVE_TO_SHOOT:
                if (pathFinished(PATH_TIMEOUT_S)) setState(State.SPINUP);
                break;

            case SPINUP:
                if (shooterStable()) {
                    spinupTimedOut = false;
                    setState(State.FEED);
                } else if (stateTimer.seconds() > SPINUP_TIMEOUT_S) {
                    spinupTimedOut = true;
                    setState(State.FEED);
                }
                break;

            case FEED:
                if (stateTimer.seconds() > FEED_PULSE_S) {
                    setFeeder(0);
                    shotsFired++;
                    if (shotsFired >= SHOTS_PER_CYCLE) {
                        finishShooting();
                    } else {
                        setState(State.RECOVER);
                    }
                }
                break;

            case RECOVER:
                if (shooterStable() || stateTimer.seconds() > RECOVER_TIMEOUT_S) setState(State.FEED);
                break;

            case DRIVE_TO_PICKUP:
                if (pathFinished(PATH_TIMEOUT_S)) setState(State.INTAKE_SWEEP);
                break;

            case INTAKE_SWEEP:
                if (pathFinished(SWEEP_TIMEOUT_S)) setState(State.INTAKE_DWELL);
                break;

            case INTAKE_DWELL:
                if (stateTimer.seconds() > INTAKE_DWELL_S) setState(State.DRIVE_TO_SHOOT);
                break;

            case PARK:
                if (pathFinished(PATH_TIMEOUT_S)) setState(State.DONE);
                break;

            case DONE:
            default:
                break;
        }
    }

    // Runs entry actions for the next state
    private void setState(State next) {
        state = next;
        stateTimer.reset();

        switch (next) {
            case DRIVE_TO_SHOOT:
                shotsFired = 0;
                setIntake(0);
                setShooter(true);
                startPath(plan.shoot);
                break;

            case SPINUP:
                setShooter(true);
                break;

            case FEED:
                setShooter(true);
                setFeeder(FEEDER_POWER);
                setIntake(INTAKE_POWER);
                break;

            case RECOVER:
                setFeeder(0);
                setIntake(0);
                break;

            case DRIVE_TO_PICKUP:
                setIntake(INTAKE_POWER);
                startPath(plan.pickupStart);
                break;

            case INTAKE_SWEEP:
                setIntake(INTAKE_POWER);
                setShooter(true);
                startPath(plan.pickupEnd);
                break;

            case INTAKE_DWELL:
                setIntake(INTAKE_POWER);
                break;

            case PARK:
                stopMechanisms();
                startPath(plan.park);
                break;

            case DONE:
                stopMechanisms();
                lastPose = follower.pose();
                break;
        }
    }

    // Decides what follows the shooting sequence
    private void finishShooting() {
        setFeeder(0);
        setIntake(0);
        setShooter(false);
        boolean timeForCycle = (AUTO_TOTAL_S - matchTimer.seconds()) >= MIN_CYCLE_TIME_S;
        if (cycle == 0 && timeForCycle) {
            cycle = 1;
            setState(State.DRIVE_TO_PICKUP);
        } else {
            setState(State.PARK);
        }
    }

    // Returns true when the path is done or has timed out
    private boolean pathFinished(double timeoutS) {
        return !follower.isBusy() || stateTimer.seconds() > timeoutS;
    }

    // Builds and starts a straight path from the current pose
    private void startPath(Pose target) {
        Pose current = follower.pose();
        follower.follow(line(current, target).linear(current, target));
    }

    // Tracks how long the shooter has stayed inside tolerance
    private void trackShooterReady() {
        if (shooter == null || Math.abs(shooterVel - SHOOTER_TARGET_VEL) > SHOOTER_TOLERANCE) {
            readyTimer.reset();
        }
    }

    // True when the shooter held tolerance long enough
    private boolean shooterStable() {
        return shooter != null && readyTimer.seconds() >= READY_HOLD_S;
    }

    // Refreshes shooter feedforward when the battery sags
    private void refreshVoltageComp() {
        if (voltageTimer.seconds() < VOLTAGE_REFRESH_S) return;
        voltageTimer.reset();
        batteryVoltage = readVoltage();
        if (Math.abs(batteryVoltage - appliedVoltage) < VOLTAGE_DEADBAND) return;
        double scale = NOMINAL_VOLTAGE / Math.max(batteryVoltage, 6.0);
        scale = Math.max(MIN_COMP_SCALE, Math.min(MAX_COMP_SCALE, scale));
        applyShooterPidf(scale);
        appliedVoltage = batteryVoltage;
    }

    // Writes shooter PIDF with a scaled feedforward
    private void applyShooterPidf(double fScale) {
        if (shooter == null) return;
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F * fScale));
    }

    // Reads battery voltage safely
    private double readVoltage() {
        return (voltageSensor != null) ? voltageSensor.getVoltage() : NOMINAL_VOLTAGE;
    }

    // Sets shooter on at target velocity or off
    private void setShooter(boolean on) {
        if (shooter == null) return;
        if (on) {
            shooter.setVelocity(SHOOTER_TARGET_VEL);
        } else {
            shooter.setPower(0);
        }
    }

    // Sets intake power
    private void setIntake(double power) {
        if (intake != null) intake.setPower(power);
    }

    // Sets feeder power
    private void setFeeder(double power) {
        if (feeder != null) feeder.setPower(power);
    }

    // Stops all mechanisms
    private void stopMechanisms() {
        setShooter(false);
        setIntake(0);
        setFeeder(0);
    }

    // Gets a motor or records it as missing
    private DcMotorEx tryMotor(String name, DcMotorSimple.Direction dir,
                               DcMotor.ZeroPowerBehavior zeroBehavior, boolean useEncoder) {
        DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
        if (motor == null) {
            missingDevices.add(name);
            return null;
        }
        motor.setDirection(dir);
        motor.setZeroPowerBehavior(zeroBehavior);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(useEncoder ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    // Starts the webcam with an AprilTag processor
    private void initVision() {
        WebcamName camera = hardwareMap.tryGet(WebcamName.class, WEBCAM_NAME);
        if (camera == null) {
            missingDevices.add(WEBCAM_NAME);
            return;
        }
        try {
            tagProcessor = new AprilTagProcessor.Builder().build();
            portal = new VisionPortal.Builder()
                    .setCamera(camera)
                    .addProcessor(tagProcessor)
                    .build();
        } catch (Exception e) {
            visionError = String.valueOf(e.getMessage());
            portal = null;
            tagProcessor = null;
        }
    }

    // Reads obelisk tags 21, 22, 23 into a motif string
    private void readMotif() {
        if (tagProcessor == null) return;
        for (AprilTagDetection detection : tagProcessor.getDetections()) {
            if (detection.id == 21) motif = "GPP";
            if (detection.id == 22) motif = "PGP";
            if (detection.id == 23) motif = "PPG";
        }
    }

    // Closes the camera portal
    private void closeVision() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
    }

    // Shows missing device and low battery warnings
    private void reportWarnings() {
        if (!missingDevices.isEmpty()) telemetry.addData("MISSING DEVICES", missingDevices);
        if (!visionError.isEmpty()) telemetry.addData("VISION ERROR", visionError);
        if (readVoltage() < LOW_VOLTAGE_WARN) telemetry.addLine("LOW BATTERY");
    }

    // Builds the plan for the selected alliance and side
    private static Plan buildPlan(Alliance a, Side s) {
        if (a == Alliance.BLUE) return makePlan(PoseFactory.degrees(), s);
        // Red mirrors the blue numbers of the opposite side
        PoseFactory red = PoseFactory.degrees().mirrorX(FIELD_MIRROR_X);
        return makePlan(red, s == Side.LEFT ? Side.RIGHT : Side.LEFT);
    }

    // Blue waypoints per side in inches and degrees - placeholders to tune at the field
    private static Plan makePlan(PoseFactory f, Side baseSide) {
        if (baseSide == Side.LEFT) {
            return new Plan(
                    f.of(26, 130, 144),
                    f.of(50, 94, 135),
                    f.of(44, 84, 180),
                    f.of(16, 84, 180),
                    f.of(40, 72, 90));
        }
        return new Plan(
                f.of(56, 8, 90),
                f.of(60, 18, 112),
                f.of(44, 36, 180),
                f.of(16, 36, 180),
                f.of(36, 12, 90));
    }

    // Formats a pose for telemetry
    private static String fmt(Pose p) {
        return String.format("x=%.1f y=%.1f h=%.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    // ===== TUNING NOTES =====
    // Run the Pedro AutoTune pages first and paste their output into pedro/Constants.java.
    // Overshoots or oscillates at path end: rerun the Foresight tuner and adjust the headingFeedback and translational controllers.
    // Stops short or brakes late: rerun the Foresight tuner and check the brake coefficients.
    // Pose drifts from reality: rerun the Two Wheel tuner for ticks to inches, directions and pod offsets.
    // Heading wrong or flipped: check the Control Hub logo and USB orientation in the Two Wheel config.
    // Shooter never reaches speed: raise SHOOTER_F first, then SHOOTER_P.
    // Shooter overshoots then hunts: lower SHOOTER_P or add a small SHOOTER_D.
    // Shooter sags after each shot: raise RECOVER_TIMEOUT_S and READY_HOLD_S.
    // Shots fall short at low battery: raise MAX_COMP_SCALE or raise SHOOTER_TARGET_VEL.
    // Pickup misses artifacts: move pickupEnd further past them and raise INTAKE_DWELL_S.
    // Feeder jams or double feeds: adjust FEED_PULSE_S and FEEDER_POWER.
    // Starts or ends in the wrong spot: retune the waypoint numbers in makePlan at the field.
    // Mechanism spins backwards: flip the matching direction constant at the top of the file.
    // Servo moves wrong way: swap SERVO_STOW_POS and SERVO_DEPLOY_POS.
}
