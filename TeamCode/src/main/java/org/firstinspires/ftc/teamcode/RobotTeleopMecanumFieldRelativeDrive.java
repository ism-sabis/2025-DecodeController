package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.BallColor;


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
@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
// @Disabled
public class RobotTeleopMecanumFieldRelativeDrive extends OpMode {
    final private RobotHardware robot = new RobotHardware();
    // Kicker auto-cycle state
    boolean kickerCycling = false;
    long kickerTimer = 0;

    // Kicker positions
    static final double KICKER_DOWN = 0.975;
    static final double KICKER_UP = 0.5; // adjust if needed

    // Timing
    static final long KICK_TIME = 200; // milliseconds for kick

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

    final float[] hsvValues = new float[3];
    boolean xPrev = false;

    BallColor[] aprilOrder = new BallColor[3];

    BallColor[] finColors = {
            BallColor.NONE, BallColor.NONE, BallColor.NONE
    };

    double driverYawOffset = 0; // initial offset in radians
    boolean squarePrev = false;   // to detect rising edge of square button

    double lastForward = 0;
    double lastRight = 0;
    double lastRotate = 0;

    float colorGain = 2.0f; // class-level
    boolean dpadRightPrev = false;
    boolean dpadLeftPrev = false;








    @Override
    public void init() {
        robot.init(hardwareMap);

        // Initialize color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        // Set initial gain
        colorSensor.setGain(colorGain);

        // Turn on light if supported
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        telemetry.addData("ColorSensor", "Initialized");

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
        telemetry.update();



        double driverYawOffset = Math.PI; // adjust to your driver position

        robot.kicker.setPosition(KICKER_DOWN);

    }



    // Call functions here
    // Place actual instructions here
    @Override
    public void loop() {

        // Limelight3A
        LLStatus status = robot.limelight.getStatus();
        telemetry.addData("Name", status.getName());
        telemetry.addData("LL", "Temp: " + JavaUtil.formatNumber(status.getTemp(), 1) + "C, CPU: "
                + JavaUtil.formatNumber(status.getCpu(), 1) + "%, FPS: " + Math.round(status.getFps()));
        telemetry.addData("Pipeline",
                "Index: " + status.getPipelineIndex() + ", Type: " + status.getPipelineType());
        LLResult result = robot.limelight.getLatestResult();
        if (result != null) {
            // Access general information.
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            telemetry.addData("PythonOutput", JavaUtil.makeTextFromList(result.getPythonOutput(), ","));
            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());
            telemetry.addData("Botpose", botpose.toString());
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            // Access fiducial results.
            for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()) {
                telemetry.addData("Fiducial",
                        "ID: " + fiducialResult.getFiducialId() + ", Family: " + fiducialResult.getFamily()
                                + ", X: " + JavaUtil.formatNumber(fiducialResult.getTargetXDegrees(), 2) + ", Y: "
                                + JavaUtil.formatNumber(fiducialResult.getTargetYDegrees(), 2));
                // Access color results.
                for (LLResultTypes.ColorResult colorResult : result.getColorResults()) {
                    telemetry.addData("Color", "X: " + JavaUtil.formatNumber(colorResult.getTargetXDegrees(), 2)
                            + ", Y: " + JavaUtil.formatNumber(colorResult.getTargetYDegrees(), 2));
                }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
        telemetry.update();

        BallColor current = detectColor();





        // Toggle light on X press
        boolean xNow = gamepad1.x;
        if (xNow && !xPrev) {
            if (colorSensor instanceof SwitchableLight) {
                SwitchableLight light = (SwitchableLight) colorSensor;
                light.enableLight(!light.isLightOn());
            }
        }
        xPrev = xNow;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        int red = (int) (colors.red * 255);
        int green = (int) (colors.green * 255);
        int blue = (int) (colors.blue * 255);
        int alpha = (int) (colors.alpha * 255);

        //telemetry.addData("Red", red);
        //telemetry.addData("Green", green);
        //telemetry.addData("Blue", blue);
        //telemetry.addData("Alpha", alpha);
        //telemetry.update();
        ;

        // Convert to HSV
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addData("Current Ball", current); // shows NONE, GREEN, or PURPLE
        telemetry.addData("Fin Colors",
                "0: " + finColors[0] + " 1: " + finColors[1] + " 2: " + finColors[2]);

        // Show distance if supported
        if (colorSensor instanceof DistanceSensor) {
            double dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            telemetry.addData("Distance (cm)", "%.2f", dist);
        }

        //telemetry.addLine("Press A to reset Yaw");
        //telemetry.addLine("Hold left bumper to drive in robot relative");
        //telemetry.addLine("The left joystick sets the robot direction");
       // telemetry.addLine("Moving the right joystick left and right turns the robot");

        double yawDeg = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
       // telemetry.addData("Yaw (deg)", yawDeg);


        // ===== TELEMETRY FOR DEBUGGING =====
        //telemetry.addLine("===== DRIVER-RELATIVE DEBUG =====");
        //telemetry.addData("driverYawOffset (deg)", Math.toDegrees(driverYawOffset));
        //telemetry.addData("robotYaw (deg)", Math.toDegrees(robotYaw));
        //telemetry.addData("joystickAngle (deg)", Math.toDegrees(Math.atan2(forward, right)));
        //telemetry.addData("theta rotated (deg)", Math.toDegrees(theta));
        //telemetry.addData("newForward", newForward);
        //telemetry.addData("newRight", newRight);



        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.triangle) {
            robot.imu.resetYaw();
        }
        /*
        // If you press the left bumper, you get a drive from the point of view of the
        // robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

         */

        // Current button states
        boolean dpadRightNow = gamepad2.dpad_right;
        boolean dpadLeftNow  = gamepad2.dpad_left;

// Increase gain on right D-pad press (rising edge)
        if (dpadRightNow && !dpadRightPrev) {
            colorGain += 0.05f;
        }

// Decrease gain on left D-pad press (rising edge)
        if (dpadLeftNow && !dpadLeftPrev && colorGain > 0.1f) {
            colorGain -= 0.05f;
        }

// Apply gain to the color sensor
        colorSensor.setGain(colorGain);

// Save previous states for rising edge detection
        dpadRightPrev = dpadRightNow;
        dpadLeftPrev  = dpadLeftNow;

// Telemetry
        telemetry.addData("Color Sensor Gain", colorGain);



        // Rising edge detection for square button
        boolean squareNow = gamepad1.square;
        if (squareNow && !squarePrev) {
            driverYawOffset += Math.PI / 2;
            driverYawOffset = driverYawOffset % (2 * Math.PI);

            // Immediately recalc motor powers using last joystick
            applyDriverOffset(driverYawOffset);
        }
        squarePrev = squareNow;

        // Rising edge detection variables
        boolean aPrev = false;
        boolean bPrev = false;
        boolean yPrev = false;
        boolean xPrev2 = false; // separate from Gamepad1 X


            // --- Gamepad2 button actions ---
            // A button → shoot one ball
            if (gamepad2.triangle && !aPrev) {
                shootOneBall();
            }
            aPrev = gamepad2.triangle;

            // B button → macro simple shoot
            if (gamepad2.square && !bPrev) {
                macroSimpleShoot();
            }
            bPrev = gamepad2.square;

            // Y button → macro randomized shoot
            if (gamepad2.x && !yPrev) {
                macroRandomizedShoot();
            }
            yPrev = gamepad2.x;

            // X button → rotate to RED (example)
            if (gamepad2.circle && !xPrev2) {
                rotateToColor(BallColor.GREEN); // or add logic to choose color dynamically
            }
            xPrev2 = gamepad2.circle;





        // Set driver orientation (angle between driver forward and field forward)
// Example: driver on south side facing north = 180 degrees


        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x); // robot-relative
        } else {
            driveDriverRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, driverYawOffset);
        }



        // Heading lock logic
        if (gamepad1.left_bumper) {
            if (!headingLocked) {
                // Store the current IMU yaw once
                lockedHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                headingLocked = true;
            }
        } else {
            headingLocked = false; // release heading lock
        }



        robot.launcher.setPower(gamepad2.right_trigger);

        GenevaStatus genevaStatus = getGenevaStatus(robot.feedingRotation);

        updateFinColor();

        // Kicker
        // Kicker auto-cycle logic (tap to fire)
        if (!kickerCycling && gamepad2.x) {
            // Start the cycle
            kickerCycling = true;
            robot.kicker.setPosition(KICKER_UP);
            kickerTimer = System.currentTimeMillis();
        }

        updateKicker();

        /*
         * if (status.inGap) {
         * robot.kicker.setPosition(0.8 - gamepad2.left_trigger * 0.3);
         * } else {
         * robot.kicker.setPosition(0.8);
         * }
         */

        lift();

        feeding();

        turret();

        LLResult limelightResult = robot.limelight.getLatestResult();
        // TODO: Fix Limelight code
        // if (limelightResult != null) {
        // double tx = limelightResult.getTx();
        // double ty = limelightResult.getTy();

        // final double targetHeight = 65;
        // final double mountAngle = 90;

        // // 1. Rotate turret
        // double turretPower = (Math.abs(tx) < 1.0) ? 0 : (0.01 * tx);
        // robot.turretSpinner.setPower(turretPower);

        // // 2. Calculate distance & launcher speed
        // double distance = (targetHeight - robot.limelightHeight) /
        // Math.tan(Math.toRadians(ty + mountAngle));
        // double launchAngle = Math.toRadians(45); // or your tuned angle
        // double velocity = Math.sqrt(9.81 * distance * distance /
        // (2 * (targetHeight - robot.limelightHeight - distance *
        // Math.tan(launchAngle))
        // * Math.pow(Math.cos(launchAngle), 2)));

        // double launcherTicksPerSec = velocity / (2 * Math.PI *
        // robot.launcherWheelRadius) * robot.motorTicksPerRev * robot.gearRatio;
        // robot.launcher.setVelocity(launcherTicksPerSec);
        // }

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
        double newRight   = r * Math.cos(theta);

        // Send to mecanum drive
        drive(newForward, newRight, rotate);
    }

    // Call this when the square button is pressed to immediately recalc
    private void applyDriverOffset(double driverYawOffset) {
        driveDriverRelative(lastForward, lastRight, lastRotate, driverYawOffset);
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
            telemetry.update();
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
        robot.feedingRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.feedingRotation.setTargetPosition((int) targetTicks);
        robot.feedingRotation.setPower(1.0);

        // Wait until done (non-blocking alternative inside loop if you prefer)
        //while (opModeIsActive() && robot.feedingRotation.isBusy()) {
        // optional safety timeout
        //}

        robot.feedingRotation.setPower(0);
        robot.feedingRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private BallColor detectColor() {
        // Read normalized colors
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Convert to 0–255
        int r = (int)(colors.red * 255);
        int g = (int)(colors.green * 255);
        int b = (int)(colors.blue * 255);

        // Tuned thresholds
        boolean isGreen = (r < 100 && g > 150 && b < 120);
        boolean isPurple = (r > 160 && g < 120 && b > 200);

        if (isGreen) {
            return BallColor.GREEN;
        } else if (isPurple) {
            return BallColor.PURPLE;
        } else {
            return BallColor.NONE;
        }
    }





    public void shootOneBall() {
        // Spin flywheel up
        robot.launcher.setPower(1.0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // restore interrupted status
        }
        // change to your real spin-up time

        // Fire
        robot.kicker.setPosition(KICKER_UP);
        try {
            Thread.sleep(130);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // restore interrupted status
        }

        robot.kicker.setPosition(KICKER_DOWN);

        // ===============================
        // TUNEABLE PAUSE BETWEEN SHOTS
        // ===============================
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // restore interrupted status
        }
        // <---- adjust this value
    }

    public void macroRandomizedShoot() {
        for (int i = 0; i < 3; i++) {
            rotateToColor(aprilOrder[i]);
            shootOneBall();
        }
    }

    public void macroSimpleShoot() {
        for (int i = 0; i < 3; i++) {
            shootOneBall();
        }
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
    public void updateFinColor() {
        // Read current color
        BallColor current = detectColor();

        // Get the Geneva fin currently at the sensor
        GenevaStatus status = getGenevaStatus(robot.feedingRotation);
        int finIndex = status.fin;

        // If a new ball just arrived (rising edge)
        if (current != BallColor.NONE && lastDetected == BallColor.NONE) {
            // Assign color to that fin
            finColors[finIndex] = current;
        }


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

    public void feeding() {

        // Kicker must be down/safe
        boolean kickerDown = robot.kicker.getPosition() >= 0.79;

        boolean feederUp = gamepad2.dpad_up;
        boolean feederDown = gamepad2.dpad_down;

        if (feederUp || feederDown) {
            if (kickerDown) {
                // SAFE → allow rotation
                robot.feedingRotation.setPower(feederUp ? 1 : -1);
            } else {
                // NOT safe → block movement + rumble
                robot.feedingRotation.setPower(0);

                // One short rumble on both sides
                gamepad1.rumble(1, 1, 300);
            }
        } else {
            robot.feedingRotation.setPower(0);
        }
    }

    public void turret() {
        if (gamepad2.left_bumper) {
            robot.turretSpinner.setPower(1);
        } else if (gamepad2.right_bumper) {
            robot.turretSpinner.setPower(-1);
        } else {
            robot.turretSpinner.setPower(0);
        }
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

        // Check launcher speed
        boolean launcherAtSpeed = robot.launcher.getPower() >= LAUNCHER_MIN_POWER;

        // Start cycle only if:
        // - Button pressed (tap)
        // - Not already cycling
        // - Launcher is up to speed
        if (!kickerCycling && gamepad2.x && launcherAtSpeed) {
            kickerCycling = true;
            robot.kicker.setPosition(KICKER_UP);
            kickerTimer = System.currentTimeMillis();
        }

        // Cycle already started
        if (kickerCycling) {
            if (System.currentTimeMillis() - kickerTimer >= KICK_TIME) {
                robot.kicker.setPosition(KICKER_DOWN);
                kickerCycling = false;
            }
        }
    }

}
