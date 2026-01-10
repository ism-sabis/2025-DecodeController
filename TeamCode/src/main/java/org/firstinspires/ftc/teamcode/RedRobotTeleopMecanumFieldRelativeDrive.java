package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RTPAxon;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.GamepadPair;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Red Robot: Field Relative Mecanum Drive", group = "Robot")
// @Disabled
public class RedRobotTeleopMecanumFieldRelativeDrive extends OpMode {
    final private RobotHardware robot = new RobotHardware();

    GamepadPair gamepads;

    // Kicker auto-cycle state
    boolean kickerCycling = false;
    long kickerTimer = 0;

    // Kicker positions
    static final double KICKER_DOWN = 0.725;
    static final double KICKER_UP = 0.5; // adjust if needed

    // Timing
    static final long KICK_TIME = 500; // milliseconds for kick

    // Launcher speed threshold (adjust)
    static final double LAUNCHER_MIN_POWER = 0.1; // normalized 0 to 1.0

    static final double LIFT_SPEED = 1;

    BallColor lastDetected = BallColor.NONE;

    // Heading lock
    private double lockedHeading = 0.0;
    private boolean headingLocked = false;

    // Slow mode
    private double maxSpeed = 1.0; // default full speed

    // Color Sensor
    NormalizedColorSensor colorSensor;

    NormalizedColorSensor colorSensor1;

    final float[] hsvValues = new float[3];
    boolean xPrev = false;

    BallColor[] aprilOrder = new BallColor[3];

    BallColor[] finColors = {
            BallColor.NONE, BallColor.NONE, BallColor.NONE
    };

    double driverYawOffset = 0; // initial offset in radians
    boolean squarePrev = false; // to detect rising edge of square button

    double lastForward = 0;
    double lastRight = 0;
    double lastRotate = 0;

    float colorGain = 12.65f; // class-level
    boolean dpadRightPrev = false;
    boolean dpadLeftPrev = false;

    boolean lsButtonPreviouslyPressed = false;

    long intakeColorIgnoreUntil = 0;

    private boolean aprilOrderSet = false;

    double distanceHypotenuse = 0;

    double distance = 0;

    double distanceNew = 0;

    double testingLauncherPower = 0.5;

    int feederState = 0; // -1 = backward, 0 = off, 1 = forward

    // Indexer slot tracking
    static final int NUM_SLOTS = 3;
    BallColor[] indexerSlots = {BallColor.NONE, BallColor.NONE, BallColor.NONE};
    int indexerAt = 0; // Which slot is at shoot position (0 = shoot pos)
    static final double ANGLE_PER_SLOT = 120.0; // degrees to rotate per slot

    // Intake management
    boolean intakeActive = false;
    long intakeStartTime = 0;
    static final long INTAKE_SPIN_TIME = 2000; // 2 seconds to intake one ball

    private RTPAxon servo;
    private RTPAxon servo1;

    private LauncherState launcherState;

    ElapsedTime waitTimer = new ElapsedTime();

    @Override
    public void init() {

        gamepads = new GamepadPair(gamepad1, gamepad2);

        launcherState = LauncherState.IDLE;

        robot.init(hardwareMap);

        // Initialize color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");

        // Set initial gain
        colorSensor.setGain(colorGain);
        colorSensor1.setGain(colorGain);

        CRServo crservo = hardwareMap.crservo.get("indexer");
        CRServo crservo1 = hardwareMap.crservo.get("indexer1");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "indexerEncoder");
        AnalogInput encoder1 = hardwareMap.get(AnalogInput.class, "indexerEncoder1");
        servo = new RTPAxon(crservo, encoder);
        servo1 = new RTPAxon(crservo1, encoder1);
        servo1.setRtp(false);

        telemetry.addData("ColorSensor", "Initialized");
        telemetry.addData("ColorSensor1", "Initialized");

        // Limelight3A
        // LLStatus status;
        // LLResult result;
        // Pose3D botpose;
        // double captureLatency;
        // double targetingLatency;
        // List<LLResultTypes.FiducialResult> fiducialResults;
        // LLResultTypes.FiducialResult fiducialResult;
        // LLResultTypes.FiducialResult fiducialResult;
        // List<LLResultTypes.ColorResult> colorResults;
        // LLResultTypes.ColorResult colorResult;

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData(">", "Robot Ready.  Press Play.");
        // telemetry.update();

        double driverYawOffset = Math.PI; // adjust to your driver position

        robot.kicker.setPosition(KICKER_DOWN);

    }

    // Call functions here
    // Place actual instructions here
    @Override
    public void loop() {

        if (!robot.launcher.isBusy()) {
            servo.update();
        }
        double followerScale = 0.5;

        servo1.setRawPower(servo.getPower() * followerScale);

        gamepads.copyStates();

        // Set the default debounce time for all buttons
        gamepads.setDebounceTime(250); // 250ms debounce time

        // Update shooter color and LED
        updateShooterPosColor();
        updateShooterLed();

        // Handle intake auto-stop
        if (intakeActive) {
            long elapsed = System.currentTimeMillis() - intakeStartTime;
            if (elapsed > INTAKE_SPIN_TIME) {
                stopIntake();
                // macroReindexIdentifyColors();
            }
        }

        // ========== 3-LAYER GAMEPAD CONTROL ==========
        boolean ltHeld = gamepad2.left_trigger > 0.4;
        boolean rtHeld = gamepad2.right_trigger > 0.4;

        // LAYER 3: MANUAL (RIGHT TRIGGER HELD)
        if (rtHeld) {
            handleManualControls();
        }
        // LAYER 2: ADVANCED MACROS (LEFT TRIGGER HELD)
        else if (ltHeld) {
            if (gamepads.isPressed(2, "cross")) {
                macroIntakeOneBall();
            }
            if (gamepads.isPressed(2, "circle")) {
                // Auto intake fill
                while (findFirstEmptySlot() != -1) {
                    macroIntakeOneBall();
                    try {
                        Thread.sleep(3000);
                    } catch (InterruptedException e) {
                    }
                }
            }
            if (gamepads.isPressed(2, "square")) {
                macroReindexIdentifyColors();
            }
            if (gamepads.isPressed(2, "triangle")) {
                // Eject closest ball
                rotateIndexerTo((indexerAt + 1) % NUM_SLOTS);
            }
        }
        // LAYER 1: BASIC MACROS (NO TRIGGER)
        else {
            if (gamepads.isPressed(2, "cross")) {
                macroIntakeOneBall();
            }
            if (gamepads.isPressed(2, "circle")) {
                macroShootOneBall(BallColor.GREEN);
            }
            if (gamepads.isPressed(2, "square")) {
                macroShootOneBall(BallColor.PURPLE);
            }
            if (gamepads.isPressed(2, "triangle")) {
                macroShootAllBalls();
            }
            if (gamepads.isPressed(2, "dpad_up")) {
                macroShootInPattern();
            }
        }

        telemetry.addData("launcherPower", "%.2f", testingLauncherPower);

        // ========== REST OF YOUR EXISTING CODE ==========

        // Limelight (keep your existing code)
        LLStatus status = robot.limelight.getStatus();
        LLResult result = robot.limelight.getLatestResult();
        if (result != null) {
            for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()) {
                telemetry.addData("Fiducial", "ID: " + fiducialResult.getFiducialId());
                distanceNew = getDistanceFromTag(result.getTa());

                double targetOffsetAngle_Vertical = 0.0;

                targetOffsetAngle_Vertical = result.getTy();
                // how many degrees back is your limelight rotated from perfectly vertical?
                double limelightMountAngleDegrees = 0;

                // distance from the center of the Limelight lens to the floor
                double limelightLensHeightInches = 16.06;

                // distance from the target to the floor
                double goalHeightInches = 29.5;

                double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

                // calculate distance
                double distanceHypotenuse = (goalHeightInches - limelightLensHeightInches)
                        / Math.sin(angleToGoalRadians);

                double distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

                // telemetry.addData("AprilTag distanceHypotenuse (in)", "%.2f",
                // distanceHypotenuse);
                // telemetry.addData("AprilTag distance (in)", "%.2f", distance);
                telemetry.addData("AprilTag distance new (in)", "%.2f", distanceNew);
                // telemetry.addData("ty (deg)", "%.2f", targetOffsetAngle_Vertical);
                // telemetry.addData("Angle (deg)", "%.2f", angleToGoalDegrees);
                double area = result.getTa();
                telemetry.addData("area", "%.2f", area);
            }
        }

        displayAprilTagOrder();

        BallColor current1 = detectColor1();
        // telemetry.addData("Current Ball Sensor1", current1);

        if (current1 == BallColor.GREEN || current1 == BallColor.PURPLE) {
            gamepad2.rumble(1, 1, 300);
        }

        // Drive
        if (gamepad1.left_bumper) {
            driveDriverRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, driverYawOffset);
        } else {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // Turret
        updateTurretControl();

        // Lift
        // lift();

        shootLoop();

        // Kicker (keep existing logic, or manual in layer 3 will override)
        updateKicker();


        // Telemetry
        telemetry.addData("spinUpTime", calculateSpinUpTime());
        telemetry.addData("launcherPower",calculateLauncherPower());
        telemetry.addData("RPM", calculateRPM());
        telemetry.addData("Indexer At", indexerAt);
        telemetry.addData("Slots", indexerSlots[0] + " | " + indexerSlots[1] + " | " + indexerSlots[2]);
        telemetry.addData("launcherState", launcherState.toString());
        telemetry.update();

    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        // theta = AngleUnit.normalizeRadians(theta -
        // robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double yaw = headingLocked ? lockedHeading : robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        theta = AngleUnit.normalizeRadians(theta - yaw);

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // ============================================
    // INDEXER & BALL TRACKING HELPERS
    // ============================================

    void updateShooterPosColor() {
        BallColor detected = detectColor1();
        indexerSlots[indexerAt] = detected;
    }

    void updateShooterLed() {
        BallColor color = indexerSlots[indexerAt];
        if (color == BallColor.GREEN) {
            gamepads.setLed(2, 0.0, 1.0, 0.0); // Green LED
        } else if (color == BallColor.PURPLE) {
            gamepads.setLed(2, 0.6, 0.0, 1.0); // Purple LED
        } else {
            gamepads.setLed(2, 0.0, 0.0, 0.0); // Off
        }
    }

    int findNearestSlotWithColor(BallColor target) {
        int bestDist = NUM_SLOTS;
        int bestIdx = -1;
        for (int i = 0; i < NUM_SLOTS; i++) {
            if (indexerSlots[i] == target) {
                // Calculate shortest rotational distance
                int forward = (i - indexerAt + NUM_SLOTS) % NUM_SLOTS;
                int backward = (indexerAt - i + NUM_SLOTS) % NUM_SLOTS;
                int d = Math.min(forward, backward);

                if (d < bestDist) {
                    bestDist = d;
                    bestIdx = i;
                }
            }
        }
        return bestIdx;
    }

    int findFirstEmptySlot() {
        for (int i = 0; i < NUM_SLOTS; i++) {
            if (indexerSlots[i] == BallColor.NONE) {
                return i;
            }
        }
        return -1;
    }

    void rotateIndexerTo(int targetIdx) {
        /*
         * if (targetIdx < 0 || targetIdx >= NUM_SLOTS) return;
         *
         * int delta = (targetIdx - indexerAt + NUM_SLOTS) % NUM_SLOTS;
         * double newTarget = servo.getTotalRotation() + (delta * ANGLE_PER_SLOT);
         *
         * servo.setTargetRotation(newTarget);
         * servo1.setTargetRotation(newTarget);
         * robot.feedingRotation.setPower(1.0);
         * indexerAt = targetIdx;
         */
        // Compute forward delta (0,1,2)
        int delta = (targetIdx - indexerAt + NUM_SLOTS) % NUM_SLOTS;

        // Convert to degrees
        double degreesToMove = delta * ANGLE_PER_SLOT;

        // Command servo to move relative
//        servo.changeTargetRotation(degreesToMove);

        // Update current slot
        indexerAt = targetIdx;
    }

    boolean isIndexerAtTarget(double tolerance) {
        return servo.isAtTarget(tolerance) && servo1.isAtTarget(tolerance);
    }

    // ============================================
    // LAUNCHER POWER CALCULATION (DISTANCE + VOLTAGE)
    // ============================================

    /*
     * double calculateLauncherPower() {
     * LLResult result = robot.limelight.getLatestResult();
     * if (result == null) return 1;
     *
     * for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
     * if (tag.getFiducialId() == 24) { // Goal tag
     * double ty = tag.getTargetYDegrees();
     *
     * // Estimate distance based on angle
     * double estimatedDistance = 72.0 / Math.tan(Math.toRadians(ty + 45));
     *
     * // Get battery voltage
     * double voltage = robot.voltageSensor.getVoltage();
     * double nominalVoltage = 13.0;
     *
     * // Power scales with distance and voltage compensation
     * double basePower = 0.8 + (estimatedDistance / 200.0);
     * double voltageFactor = nominalVoltage / voltage;
     * double finalPower = Math.min(1.0, basePower * voltageFactor);
     *
     * telemetry.addData("Launcher Distance", estimatedDistance);
     * telemetry.addData("Battery Voltage", voltage);
     * telemetry.addData("Launcher Power", finalPower);
     *
     * return finalPower;
     * }
     * }
     * return 1;
     * }
     */

    public double getDistanceFromTag(double ta) {
        distanceNew = 72.34359 * Math.pow(ta, -0.479834);
        return distanceNew;
    }

    double calculateLauncherPower() {
        LLResult result = robot.limelight.getLatestResult();
        if (result == null)
            return 1;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() != 24)
                continue;

            // double ty = Math.max(tag.getTargetYDegrees(), 2.0);

            double area = result.getTa();
            // telemetry.addData("area", "%.2f", area);

            double launcherPower = 0;

            launcherPower = (0.00303584 * distanceNew) + 0.586525;

            return launcherPower;
        }

        return 1;
    }

    double calculateRPM(){
        double launcherRPM = 0;
        launcherRPM = 6000 * calculateLauncherPower();
        return launcherRPM;
    }

    double calculateSpinUpTime(){
        double spinUpTime = 0;
        spinUpTime = 0.00000275688 * Math.pow(calculateRPM(), 1.54859);
        return spinUpTime;
    }

    // ============================================
    // MACRO FUNCTIONS
    // ============================================

    void macroIntakeOneBall() {
        int emptySlot = findFirstEmptySlot();
        if (emptySlot == -1) {
            gamepads.blipRumble(2, 2); // Vibrate: full
            telemetry.addLine("INTAKE:  Indexer full!");
            return;
        }

        rotateIndexerTo(emptySlot);

        // Wait for indexer
        long startWait = System.currentTimeMillis();
        while (System.currentTimeMillis() - startWait < 1500 && !isIndexerAtTarget(5)) {
            servo.update();
        }

        // Start intake
        // robot.indexer.setPower(1.0);
        // robot.indexer1.setPower(1.0);
        // robot.feedingRotation.setPower(1.0);
        // intakeActive = true;
        // intakeStartTime = System.currentTimeMillis();
    }

    void stopIntake() {
        servo.setPower(0);
        servo1.setPower(0);
        robot.feedingRotation.setPower(0);
        intakeActive = false;
    }

    void macroShootOneBall(BallColor color) {
        int idx = findNearestSlotWithColor(color);
        if (idx == -1) {
            gamepads.blipRumble(2, 3); // Vibrate: not found
            telemetry.addLine("SHOOT: " + color + " not found!");
            return;
        }

        robot.feedingRotation.setPower(1.0);
        rotateIndexerTo(idx);

        // Wait for rotation
        long startWait = System.currentTimeMillis();
        while (System.currentTimeMillis() - startWait < 1500 && !isIndexerAtTarget(5)) {
            servo.update();
        }

        // Shoot
        /*
         * spinLauncherToSetPower();
         * try { Thread.sleep(10000); } catch (InterruptedException e) { }
         *
         * safeKick();
         * try { Thread.sleep(500); } catch (InterruptedException e) { }
         * robot.kicker.setPosition(KICKER_DOWN);
         * robot.launcher.setPower(0);
         */
//        shoot();

        robot.feedingRotation.setPower(0);

        indexerSlots[indexerAt] = BallColor.NONE;
    }

    void shootLoop() {
        if (launcherState == LauncherState.STARTING) {
            double power = calculateLauncherPower();
            robot.launcher.setPower(power);
            waitTimer.reset();
            launcherState = LauncherState.SPINNING;
        } else if (launcherState == LauncherState.SPINNING && waitTimer.seconds() >= calculateSpinUpTime()) {
            //safeKick();
            robot.kicker.setPosition(KICKER_UP);
            waitTimer.reset();
            launcherState = LauncherState.KICKING;
        } else if (launcherState == LauncherState.KICKING && waitTimer.seconds() >= 0.5) {
            robot.kicker.setPosition(KICKER_DOWN);
            launcherState = LauncherState.UNKICKING;
        }
        else if (launcherState == LauncherState.UNKICKING) {
            robot.launcher.setPower(0);
            // Treat current position as the new "correct" position
            double currentPos = servo.getTotalRotation();
            servo.setTargetRotation(currentPos);
            servo.resetPID();
            launcherState = LauncherState.IDLE;
        }
    }


    void testingShootOneBall() {
        // Shoot
        robot.launcher.setPower(testingLauncherPower);
        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
        }
        safeKick();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
        }
        robot.kicker.setPosition(KICKER_DOWN);

        robot.launcher.setPower(0);

    }

    void macroShootAllBalls() {
        robot.feedingRotation.setPower(1.0);
        for (int count = 0; count < NUM_SLOTS; count++) {
            BallColor c = indexerSlots[indexerAt];
            if (c != BallColor.NONE) {
//                shoot();
                indexerSlots[indexerAt] = BallColor.NONE;
            }
            if (count < NUM_SLOTS - 1) {
                rotateIndexerTo((indexerAt + 1) % NUM_SLOTS);
                try {
                    Thread.sleep(800);
                } catch (InterruptedException e) {
                }
            }
        }
        robot.feedingRotation.setPower(0);
        gamepads.blipRumble(2, 2); // Done!
    }

    void macroShootInPattern() {
        for (int i = 0; i < NUM_SLOTS; i++) {
            BallColor toShoot = aprilOrder[i];
            if (toShoot == BallColor.NONE)
                continue;
            macroShootOneBall(toShoot);
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
            }
        }
        gamepads.blipRumble(2, 2);
    }

    void macroReindexIdentifyColors() {
        for (int spin = 0; spin < NUM_SLOTS; spin++) {
            updateShooterPosColor();
            robot.feedingRotation.setPower(0.7);
            rotateIndexerTo((indexerAt + 1) % NUM_SLOTS);
            try {
                Thread.sleep(600);
            } catch (InterruptedException e) {
            }
        }
        robot.feedingRotation.setPower(0);
    }

    // ============================================
    // MANUAL CONTROLS (LAYER 3 - RIGHT TRIGGER)
    // ============================================

    void handleManualControls() {
        if (gamepad2.right_trigger < 0.4)
            return;

        // Manual indexer rotation
        if (gamepads.isPressed(2, "cross")) {
            robot.feedingRotation.setPower(1.0);
            rotateIndexerTo((indexerAt - 1 + NUM_SLOTS) % NUM_SLOTS);
            robot.feedingRotation.setPower(0);
        }
        if (gamepads.isPressed(2, "circle")) {
            robot.feedingRotation.setPower(1.0);
            rotateIndexerTo((indexerAt + 1) % NUM_SLOTS);
            robot.feedingRotation.setPower(0);
        }

        // Manual intake
        if (gamepad2.square) {
            // robot.indexer.setPower(1.0);
            // robot.indexer1.setPower(1.0);
            robot.feedingRotation.setPower(1);
        } else if (gamepad2.triangle) {
            // robot.indexer.setPower(-1.0);
            // robot.indexer1.setPower(-1.0);
            robot.feedingRotation.setPower(-1);
        } else {
            // robot.indexer.setPower(0);
            // robot.indexer1.setPower(0);
            robot.feedingRotation.setPower(0);
        }

        // Manual launcher
        if (launcherState == LauncherState.IDLE) {
            robot.launcher.setPower(gamepad2.right_stick_y);

            // Manual kicker
            if (gamepad2.dpad_left) {
                robot.kicker.setPosition(0.55);
            }
            if (gamepad2.dpad_right) {
                robot.kicker.setPosition(KICKER_DOWN);
            }
        }

        if (gamepad2.dpad_up) {
            testingLauncherPower = (testingLauncherPower + 0.01);
        }
        if (gamepad2.dpad_down) {
            testingLauncherPower = (testingLauncherPower - 0.01);
        }
        if (gamepad2.right_stick_button) {
            testingShootOneBall();
        }
        if (gamepad2.left_stick_button && launcherState == LauncherState.IDLE) {
            launcherState = LauncherState.STARTING;
        }
    }

    private void driveDriverRelative(double forward, double right, double rotate, double driverYawOffset) {
        // Store the last joystick values
        lastForward = forward;
        lastRight = right;
        lastRotate = rotate;

        double robotYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Joystick vector → polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Apply robot yaw and driver offset
        theta = theta - robotYaw + driverYawOffset;

        // Convert back to robot-relative Cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Send to mecanum drive
        drive(newForward, newRight, rotate);
    }

    // Call this when the square button is pressed to immediately recalc
    private void applyDriverOffset(double driverYawOffset) {
        driveDriverRelative(lastForward, lastRight, lastRotate, driverYawOffset);
    }

    public void shootOneBallAlways() {
        // Spin flywheel
        robot.launcher.setPower(1.0);

        try {
            Thread.sleep(750); // spin-up time
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Fire regardless of kicker position
        robot.kicker.setPosition(KICKER_UP);

        try {
            Thread.sleep(250); // allow kick
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        robot.kicker.setPosition(KICKER_DOWN);

        // Optional small pause between shots
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void rotateToColor(BallColor desired) {

        // 1. Find which fin has the desired color
        int targetFin = -1;
        for (int i = 0; i < 3; i++) {
            if (finColors[i] == desired) {
                targetFin = i;
                break;
            }
        }

        if (targetFin == -1) {
            telemetry.addLine("ERROR: Desired color not found in any fin!");
            // telemetry.update();
            return;
        }

        // 2. Read current Geneva position
        GenevaStatus status = getGenevaStatus(robot.feedingRotation);
        int currentFin = status.fin; // 0–2

        // 3. Compute shortest direction (CW / CCW)
        int diff = targetFin - currentFin;

        // Normalize difference to −1, 0, +1 (wraparound over 3 fins)
        if (diff == 2)
            diff = -1;
        if (diff == -2)
            diff = 1;

        double direction = Math.signum(diff); // -1 → rotate backwards, +1 → forwards

        if (direction == 0) {
            // Already at correct fin
            return;
        }

        // 4. Convert fin distance to ticks
        final double TICKS_PER_REV = 5377.0;
        final double TICKS_PER_FIN = TICKS_PER_REV / 3.0;

        double targetTicks = robot.feedingRotation.getCurrentPosition() + direction * TICKS_PER_FIN;

        // 5. Rotate until you reach the target
        // robot.feedingRotation.setTargetPosition((int) targetTicks);
        // robot.feedingRotation.setPower(1.0);

        // Wait until done (non-blocking alternative inside loop if you prefer)
        // while (opModeIsActive() && robot.feedingRotation.isBusy()) {
        // optional safety timeout
        // }

        // robot.feedingRotation.setPower(0);

    }
    /*
     * public void aimTurretAtRedGoal() {
     * LLResult result = robot.limelight.getLatestResult();
     *
     * if (result == null) {
     * robot.turretSpinner.setPower(0);
     * return;
     * }
     *
     * for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
     * if (tag.getFiducialId() == 24) { // Red Alliance goal
     * double tx = tag.getTargetXDegrees();
     *
     * // Higher sensitivity (tune if necessary)
     * double kP = 0.05; // bigger than 0.01 → faster response
     * double turretPower = -kP * tx; // negative to move toward target
     *
     *
     *
     * // Clamp max power to prevent overdrive
     * turretPower = Math.max(Math.min(turretPower, 1.0), -1.0);
     *
     * // Optional deadzone for very small errors
     * if (Math.abs(tx) < 0.5) turretPower = 0;
     *
     * robot.turretSpinner.setPower(turretPower);
     *
     * telemetry.addData("Turret Tracking", "Aiming at Tag 24");
     * telemetry.addData("tx", tx);
     * telemetry.addData("Power", turretPower);
     * return;
     * }
     * }
     *
     * // No target → stop
     * robot.turretSpinner.setPower(0);
     * telemetry.addData("Turret Tracking", "No target");
     * }
     */

    public void aimTurretAtRedGoal() {
        LLResult result = robot.limelight.getLatestResult();

        if (result == null) {
            // No target → stop turret
            robot.turretSpinner.setPower(0);
            return;
        }

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() == 24) { // Red Alliance goal
                double tx = tag.getTargetXDegrees();

                // ------------------------
                // Adaptive CR servo control
                // ------------------------
                double deadband = 1.0; // increase to ignore small jitters
                double maxPower = 0.17;
                double minPower = 0.1; // lower minPower to avoid overshoot
                double kP = 0.04; // slightly lower proportional gain

                if (Math.abs(tx) < deadband) {
                    // close enough → stop
                    robot.turretSpinner.setPower(0);
                } else {
                    // scale proportional to error, clamp to min/max
                    double scaledPower = Math.abs(kP * tx);

                    // only apply minPower if scaledPower is too small
                    if (scaledPower < minPower) {
                        scaledPower = minPower;
                    } else if (scaledPower > maxPower) {
                        scaledPower = maxPower;
                    }

                    // apply direction
                    robot.turretSpinner.setPower(Math.signum(tx) * scaledPower);
                }

                telemetry.addData("Turret Tracking", "Aiming at Tag 24");
                telemetry.addData("tx", tx);
                // telemetry.addData("Power", robot.turretSpinner.getPower());
                return;
            }
        }

        // No target found → stop turret
        robot.turretSpinner.setPower(0);
        telemetry.addData("Turret Tracking", "No target");
    }

    private BallColor detectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        int r = (int) (colors.red * 255);
        int g = (int) (colors.green * 255);
        int b = (int) (colors.blue * 255);

        // Moderate wide green: require green to be reasonably strong
        if (g > 70 && g > r - 5 && g > b - 5) {
            return BallColor.GREEN;
        }
        // Keep purple detection tuned strictly
        else if (b > r + 10 && b > g + 10) {
            return BallColor.PURPLE;
        } else {
            return BallColor.NONE;
        }
    }

    private BallColor detectColor1() {
        NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();

        int r = (int) (colors1.red * 255);
        int g = (int) (colors1.green * 255);
        int b = (int) (colors1.blue * 255);

        // Moderate wide green: require green to be reasonably strong
        if (g > 70 && g > r - 5 && g > b - 5) {
            return BallColor.GREEN;
        }
        // Keep purple detection tuned strictly
        else if (b > r + 10 && b > g + 10) {
            return BallColor.PURPLE;
        } else {
            return BallColor.NONE;
        }
    }

    public void shootOneBall() {
        // Spin flywheel up
        robot.launcher.setPower(1.0);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // restore interrupted status
        }
        // change to your real spin-up time

        // Fire
        robot.kicker.setPosition(KICKER_UP);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // restore interrupted status
        }

        robot.kicker.setPosition(KICKER_DOWN);

    }

    public void macroRandomizedShoot() {
        for (int i = 0; i < 3; i++) {
            BallColor targetColor = aprilOrder[i];

            // Skip if no color assigned
            if (targetColor == BallColor.NONE)
                continue;

            // Rotate until the correct color is detected
            while (true) {
                BallColor current = detectColor1();

                if (current == targetColor) {
                    // Target color found, stop rotation
                    servo.setPower(0);
                    servo1.setPower(0);
                    feederState = 0;

                    break; // exit while loop
                } else {
                    // Rotate feeder forward to find the target
                    servo.setPower(0);
                    servo1.setPower(0);
                    feederState = 1;

                }
            }

            // Shoot the detected ball
            shootOneBall();
        }

        // Ensure feeder stops at the end
        servo.setPower(0);
        servo1.setPower(0);
        feederState = 0;

    }

    public void macroSimpleShoot() {
        // Number of balls to shoot
        int ballsToShoot = 3;

        for (int i = 0; i < ballsToShoot; i++) {
            // Rotate until a ball of color GREEN or PURPLE is detected
            while (true) {
                BallColor current = detectColor1();

                if (current == BallColor.GREEN || current == BallColor.PURPLE) {
                    // Ball detected, stop feeder rotation
                    robot.indexer.setPower(0);
                    robot.indexer1.setPower(0);
                    feederState = 0;

                    break; // exit while loop
                } else {
                    // Keep rotating forward to find the next ball
                    robot.indexer.setPower(1);
                    robot.indexer1.setPower(1);
                    feederState = 1;

                }
            }

            // Shoot the detected ball
            shootOneBall();
        }

        // Ensure feeder stops at the end
        robot.indexer.setPower(0);
        robot.indexer1.setPower(0);
        feederState = 0;

    }

    public void readAprilTagAndStoreOrder(int tagId) {
        switch (tagId) {
            case 21:
                aprilOrder[0] = BallColor.GREEN;
                aprilOrder[1] = BallColor.PURPLE;
                aprilOrder[2] = BallColor.GREEN;
                break;

            case 22:
                aprilOrder[0] = BallColor.GREEN;
                aprilOrder[1] = BallColor.GREEN;
                aprilOrder[2] = BallColor.GREEN;
                break;

            case 23:
                aprilOrder[0] = BallColor.NONE;
                aprilOrder[1] = BallColor.NONE;
                aprilOrder[2] = BallColor.NONE;
                break;
        }
    }

    // Call this every loop
    private void updateFinColor() {
        // Read ONLY from the new color sensor
        BallColor detected = detectColor1();

        // Determine which fin is currently under the sensor
        GenevaStatus status = getGenevaStatus(robot.feedingRotation);
        int finIndex = status.fin; // 0, 1, or 2 depending on Geneva position

        // Update only that fin
        finColors[finIndex] = detected;

    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of
        // forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        if (gamepad1.right_bumper) {
            maxSpeed = 0.1; // slow mode
        } else {
            maxSpeed = 1.0; // default full speed, make this slower for outreaches
        }

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        robot.frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        robot.frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        robot.rearLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        robot.rearRight.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void lift() {
        if (gamepad1.dpad_up) {
            robot.leftLift.setPower(LIFT_SPEED);
            robot.rightLift.setPower(LIFT_SPEED);
        } else if (gamepad1.dpad_down) {
            robot.leftLift.setPower(-LIFT_SPEED);
            robot.rightLift.setPower(-LIFT_SPEED);
        } else {
            robot.rightLift.setPower(0);
            robot.leftLift.setPower(0);
        }
    }

    public void safeKick() {
        BallColor current1 = detectColor1();
        if (current1 == BallColor.GREEN || current1 == BallColor.PURPLE) {
            robot.kicker.setPosition(KICKER_UP);
        }
    }

    public void feeding() {
        BallColor current1 = detectColor1();
        boolean kickerDown = robot.kicker.getPosition() >= 0.725;

        boolean feederUp = gamepad2.dpad_up;
        boolean feederDown = gamepad2.dpad_down;

        // -----------------------
        // LAYER 2: Manual (LT held)
        // -----------------------
        if (gamepad2.left_trigger > 0.4) {

            // Manual kicker (bypass)
            if (gamepad2.dpad_left) {
                robot.kicker.setPosition(0.55);
            } else if (gamepad2.dpad_right) {
                robot.kicker.setPosition(0.725);
            }

            // Manual feeder (requires kicker down)
            if (feederUp || feederDown) {
                if (kickerDown) {
                    if (feederUp) {
                        robot.indexer.setPower(1);
                        robot.indexer1.setPower(1);
                        feederState = 1;
                    } else {
                        robot.indexer.setPower(-1);
                        robot.indexer1.setPower(-1);
                        feederState = -1;
                    }
                } else {
                    robot.indexer.setPower(0);
                    robot.indexer1.setPower(0);
                    gamepad2.rumble(1, 1, 300); // same as before
                    feederState = 0;

                }
            } else {
                robot.indexer.setPower(0);
                robot.indexer1.setPower(0);
                feederState = 0;

            }

            return; // bypass auto layer
        }

        // -------------------------------------
        // LAYER 1: Auto (no LT held)
        // Pressing button ALWAYS starts spinning
        // AND starts a 1-second ignore period
        // -------------------------------------
        if (feederUp) {
            robot.indexer.setPower(1);
            robot.indexer1.setPower(1);
            intakeColorIgnoreUntil = System.currentTimeMillis() + 1000; // 1 sec ignore
            feederState = 1;

        }

        if (feederDown) {
            robot.indexer.setPower(-1);
            robot.indexer1.setPower(-1);
            intakeColorIgnoreUntil = System.currentTimeMillis() + 1000; // 1 sec ignore
            feederState = -1;

        }

        // -------------------------------------
        // Auto-stop when color is detected
        // but ONLY after timeout expires
        // -------------------------------------
        if (System.currentTimeMillis() > intakeColorIgnoreUntil) {
            if (current1 == BallColor.GREEN || current1 == BallColor.PURPLE) {
                robot.indexer.setPower(0);
                robot.indexer1.setPower(0);
                feederState = 0;

            }
        }
    }

    public void turret() {
        if (gamepad2.left_bumper) {
            robot.turretSpinner.setPower(-1);
        } else if (gamepad2.right_bumper) {
            robot.turretSpinner.setPower(1);
        } else {
            robot.turretSpinner.setPower(0);
        }
    }

    public void updateTurretControl() {

        // ----------------------------
        // MANUAL MODE (bumper override)
        // ----------------------------
        if (gamepad2.left_bumper) {
            robot.turretSpinner.setPower(-0.8); // rotate left
            telemetry.addData("Turret Mode", "Manual Left");
            return;
        }

        if (gamepad2.right_bumper) {
            robot.turretSpinner.setPower(0.8); // rotate right
            telemetry.addData("Turret Mode", "Manual Right");
            return;
        }

        // -----------------------------------
        // AUTO MODE (no bumpers → auto aim)
        // -----------------------------------
        aimTurretAtRedGoal(); // calls the auto-aim code
    }

    /**
     * Returns the Geneva wheel position (0–5) and whether it is in a gap (true) or
     * on a fin (false).
     */
    public static class GenevaStatus {
        public int zone; // 0–5
        public int fin; // 0–2 (true fin index)
        public boolean inGap;

        public GenevaStatus(int zone, boolean inGap) {
            this.zone = zone;
            this.fin = zone / 2; // 0–2
            this.inGap = inGap;
        }
    }

    public GenevaStatus getGenevaStatus(DcMotor feedingRotation) {
        final double TICKS_PER_REV = 5377.0; // motor + 10:1 reduction
        final int ZONES = 6;
        final int FIN_TICKS = 116; // fin width in encoder ticks

        double ticksPerZone = TICKS_PER_REV / ZONES;

        // Read encoder and normalize to 0 → TICKS_PER_REV
        int ticks = robot.feedingRotation.getCurrentPosition();
        ticks = ((ticks % (int) TICKS_PER_REV) + (int) TICKS_PER_REV) % (int) TICKS_PER_REV;

        // Compute which zone (0–5)
        int zone = (int) (ticks / ticksPerZone);

        // Compute position inside current zone
        double posInZone = ticks % ticksPerZone;

        // true = in gap, false = on fin
        boolean inGap = posInZone >= FIN_TICKS;

        return new GenevaStatus(zone, inGap);
    }

    @Override
    public void stop() {
        robot.limelight.stop();
    }

    public void updateKicker() {

        BallColor current1 = detectColor1(); // use your existing detection
        boolean launcherAtSpeed = robot.launcher.getPower() >= LAUNCHER_MIN_POWER;

        // Only allow kicker to go up when a ball color is sensed
        if (!kickerCycling
                && gamepad2.x
                && launcherAtSpeed
                && (current1 == BallColor.GREEN || current1 == BallColor.PURPLE)) {

            kickerCycling = true;
            safeKick();
            kickerTimer = System.currentTimeMillis();
        }

        if (kickerCycling) {
            if (System.currentTimeMillis() - kickerTimer >= KICK_TIME) {
                robot.kicker.setPosition(KICKER_DOWN);
                kickerCycling = false;
            }
        }
    }

    // ----- Function to detect AprilTag and display MOTIF order -----
    private void displayAprilTagOrder() {
        // Only run if we haven’t set the order yet
        if (!aprilOrderSet) {
            LLResult tagResult = robot.limelight.getLatestResult(); // get latest camera result
            if (tagResult != null) {
                List<LLResultTypes.FiducialResult> tags = tagResult.getFiducialResults(); // get detected tags

                if (!tags.isEmpty()) { // if a tag was detected
                    int detectedTagId = tags.get(0).getFiducialId(); // first tag

                    // Use your existing function to set the order
                    readAprilTagAndStoreOrder(detectedTagId);
                    aprilOrderSet = true; // lock order

                    // Show telemetry
                    telemetry.addData("AprilTag ID", detectedTagId);
                    telemetry.addData("AprilOrder",
                            "0: " + aprilOrder[0] + ", 1: " + aprilOrder[1] + ", 2: " + aprilOrder[2]);
                    // telemetry.update(); // make sure it actually shows
                }
            }
        }
    }

}
