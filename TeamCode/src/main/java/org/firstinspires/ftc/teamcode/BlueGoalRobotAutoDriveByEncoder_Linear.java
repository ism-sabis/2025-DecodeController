/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "Blue Goal Robot: Auto Drive By Encoder", group = "Robot")
// @Disabled
public class BlueGoalRobotAutoDriveByEncoder_Linear extends LinearOpMode {
    final private RobotHardware robot = new RobotHardware();
    /* Declare OpMode members. */
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;
    public DcMotor launcher = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor feedingRotation = null;
    public Servo kicker = null;
    public CRServo turretSpinner = null;
    public Limelight3A limelight = null;

    public NormalizedColorSensor colorSensor = null;
    public NormalizedColorSensor colorSensor1 = null;

    // public IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

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
    // NormalizedColorSensor colorSensor;

    // NormalizedColorSensor colorSensor1;

    final float[] hsvValues = new float[3];
    boolean xPrev = false;

    BallColor[] aprilOrder = new BallColor[3];

    BallColor[] finColors = {
            BallColor.NONE, BallColor.NONE, BallColor.NONE
    };

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

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's
    // COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth
    // spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the
    // direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.7; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.5; // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        robot.init(hardwareMap); // This will initialize motors, sensors, and limelight
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");

        // Apply gain
        colorSensor.setGain(colorGain);
        colorSensor1.setGain(colorGain);

        // To drive forward, most robots need the motor on one side to be reversed,
        // because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust
        // these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear
        // Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                rearLeft.getCurrentPosition(),
                rearRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        /*
         * // Limelight3A
         * LLStatus status = robot.limelight.getStatus();
         * // telemetry.addData("Name", status.getName());
         * //telemetry.addData("LL", "Temp: " + JavaUtil.formatNumber(status.getTemp(),
         * 1) + "C, CPU: "
         * // + JavaUtil.formatNumber(status.getCpu(), 1) + "%, FPS: " +
         * Math.round(status.getFps()));
         * //telemetry.addData("Pipeline",
         * // "Index: " + status.getPipelineIndex() + ", Type: " +
         * status.getPipelineType());
         * LLResult result = robot.limelight.getLatestResult();
         * if (result != null) {
         * // Access general information.
         * Pose3D botpose = result.getBotpose();
         * double captureLatency = result.getCaptureLatency();
         * double targetingLatency = result.getTargetingLatency();
         * //telemetry.addData("PythonOutput",
         * JavaUtil.makeTextFromList(result.getPythonOutput(), ","));
         * //telemetry.addData("tx", result.getTx());
         * //telemetry.addData("txnc", result.getTxNC());
         * //telemetry.addData("ty", result.getTy());
         * //telemetry.addData("tync", result.getTyNC());
         * //telemetry.addData("Botpose", botpose.toString());
         * //telemetry.addData("LL Latency", captureLatency + targetingLatency);
         * // Access fiducial results.
         * for (LLResultTypes.FiducialResult fiducialResult :
         * result.getFiducialResults()) {
         * telemetry.addData("Fiducial",
         * "ID: " + fiducialResult.getFiducialId() + ", Family: " +
         * fiducialResult.getFamily()
         * + ", X: " + JavaUtil.formatNumber(fiducialResult.getTargetXDegrees(), 2) +
         * ", Y: "
         * + JavaUtil.formatNumber(fiducialResult.getTargetYDegrees(), 2));
         * // Access color results.
         * for (LLResultTypes.ColorResult colorResult : result.getColorResults()) {
         * telemetry.addData("Color", "X: " +
         * JavaUtil.formatNumber(colorResult.getTargetXDegrees(), 2)
         * + ", Y: " + JavaUtil.formatNumber(colorResult.getTargetYDegrees(), 2));
         * }
         * }
         * } else {
         * telemetry.addData("Limelight", "No data available");
         * }
         * 
         * // ----- Separate AprilTag detection for MOTIF -----
         * if (!aprilOrderSet) {
         * LLResult tagResult = robot.limelight.getLatestResult();
         * if (tagResult != null) {
         * List<LLResultTypes.FiducialResult> tags = tagResult.getFiducialResults();
         * if (!tags.isEmpty()) {
         * int detectedTagId = tags.get(0).getFiducialId();
         * 
         * // Store the order for this match
         * readAprilTagAndStoreOrder(detectedTagId);
         * aprilOrderSet = true; // lock order
         * 
         * // Telemetry: show the scanned order
         * telemetry.addData("AprilTag ID", detectedTagId);
         * telemetry.addData("AprilOrder",
         * "0: " + aprilOrder[0] + ", 1: " + aprilOrder[1] + ", 2: " + aprilOrder[2]);
         * telemetry.update();
         * }
         * }
         * }
         * 
         * BallColor current1 = detectColor1();
         * telemetry.addData("Current Ball Sensor1", current1); // shows NONE, GREEN, or
         * PURPLE
         * 
         * NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
         * 
         * // Convert to HSV
         * Color.colorToHSV(colors1.toColor(), hsvValues);
         * 
         * //telemetry.addData("Current Ball", current); // shows NONE, GREEN, or PURPLE
         * telemetry.addData("Fin Colors",
         * "0: " + finColors[0] + " 1: " + finColors[1] + " 2: " + finColors[2]);
         * 
         * // Show distance if supported
         * if (colorSensor instanceof DistanceSensor) {
         * double dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
         * //telemetry.addData("Distance (cm)", "%.2f", dist);
         * }
         * // Show distance if supported
         * if (colorSensor1 instanceof DistanceSensor) {
         * double dist = ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM);
         * //telemetry.addData("Distance (cm)", "%.2f", dist);
         * }
         * 
         * // Apply gain to the color sensor
         * colorSensor.setGain(colorGain);
         * colorSensor1.setGain(colorGain);
         */
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Start in Quadrant 4

        encoderDrive(DRIVE_SPEED, 7, 7, 2); // S1: Forward toward Y-axis (mirrored), 5 sec timeout

        // encoderDrive(TURN_SPEED, 10, -10, 4.0); // S2: Turn right 12 inches (mirrored
        // left), 4 sec timeout

        encoderDrive(TURN_SPEED, 23, -23, 4.0); // S2: Turn left 50 inches (mirrored right), 4 sec timeout

        encoderDrive(DRIVE_SPEED, -12, -12, 4.0); // S3: Forward 24 inches (mirrored), 4 sec timeout

        encoderDrive(TURN_SPEED, 23, -23, 4.0); // S2: Turn right 12 inches (mirrored left), 4 sec timeout
        /*
         * shootOneBall(); // Shoot a single ball
         * 
         * //sleep(5000);
         * /*
         * int ballsToShoot = 3;
         * 
         * 
         * for (int i = 0; i < ballsToShoot; i++) {
         * // Rotate until a ball of color GREEN or PURPLE is detected
         * while (true) {
         * BallColor current = detectColor1();
         * 
         * if (current == BallColor.GREEN || current == BallColor.PURPLE) {
         * // Ball detected, stop feeder rotation
         * robot.feedingRotation.setPower(0);
         * break; // exit while loop
         * } else {
         * // Keep rotating forward to find the next ball
         * robot.feedingRotation.setPower(1);
         * }
         * }
         * }
         * 
         * shootOneBall(); // Shoot a single ball
         * 
         * //sleep(5000);
         * 
         * for (int i = 0; i < ballsToShoot; i++) {
         * // Rotate until a ball of color GREEN or PURPLE is detected
         * while (true) {
         * BallColor current = detectColor1();
         * 
         * if (current == BallColor.GREEN || current == BallColor.PURPLE) {
         * // Ball detected, stop feeder rotation
         * robot.feedingRotation.setPower(0);
         * break; // exit while loop
         * } else {
         * // Keep rotating forward to find the next ball
         * robot.feedingRotation.setPower(1);
         * }
         * }
         * }
         * 
         * 
         * 
         * encoderDrive(DRIVE_SPEED, 18, 18, 4.0); // S3: Reverse 24 inches (mirrored),
         * 4 sec timeout
         * 
         * 
         * encoderDrive(TURN_SPEED, 23, -23, 4.0); // S2: Turn right 12 inches (mirrored
         * left), 4 sec timeout
         * 
         * int ballsToShoot = 3;
         * 
         * 
         * for (int i = 0; i < ballsToShoot; i++) {
         * // Rotate until a ball of color GREEN or PURPLE is detected
         * while (true) {
         * BallColor current = detectColor1();
         * 
         * if (current == BallColor.GREEN || current == BallColor.PURPLE) {
         * // Ball detected, stop feeder rotation
         * robot.feedingRotation.setPower(0);
         * break; // exit while loop
         * } else {
         * // Keep rotating forward to find the next ball
         * robot.feedingRotation.setPower(1);
         * }
         * }
         * }
         */
        // seekFeederToColor(BallColor.GREEN);

        // encoderDrive(DRIVE_SPEED, 10, 10, 4.0); // S3: Reverse 24 inches (mirrored),
        // 4 sec timeout

        // awaitFeederColor(BallColor.GREEN);

        // encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn right 12 inches (mirrored
        // left), 4 sec timeout

        /*
         * macroSimpleShoot(); // Shoot balls based on AprilTag order
         * 
         * 
         * encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn left 12 inches (mirrored
         * right), 4 sec timeout
         * 
         * 
         * encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // S3: Forward 24 inches
         * (mirrored), 4 sec timeout
         * 
         * 
         * encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn right 12 inches (mirrored
         * left), 4 sec timeout
         * 
         * 
         * encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Reverse 24 inches (mirrored),
         * 4 sec timeout
         * 
         * 
         * encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn right 12 inches (mirrored
         * left), 4 sec timeout
         * 
         * 
         * encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Reverse 24 inches (mirrored),
         * 4 sec timeout
         * 
         * 
         * encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn right 12 inches (mirrored
         * left), 4 sec timeout
         * 
         * 
         * macroSimpleShoot(); // Shoot balls based on AprilTag order
         * 
         * 
         * encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn left 12 inches (mirrored
         * right), 4 sec timeout
         * 
         * 
         * encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // S3: Forward 24 inches
         * (mirrored), 4 sec timeout
         * 
         * 
         * encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn right 12 inches (mirrored
         * left), 4 sec timeout
         * 
         * 
         * encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Reverse 24 inches (mirrored),
         * 4 sec timeout
         * 
         * 
         * encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn right 12 inches (mirrored
         * left), 4 sec timeout
         * 
         * 
         * encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Reverse 24 inches (mirrored),
         * 4 sec timeout
         * 
         * 
         * encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn right 12 inches (mirrored
         * left), 4 sec timeout
         * 
         * 
         * macroSimpleShoot(); // Shoot balls based on AprilTag order
         * 
         * 
         * encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn left 12 inches (mirrored
         * right), 4 sec timeout
         * 
         * 
         * encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // S3: Forward 24 inches
         * (mirrored), 4 sec timeout
         */

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); // pause to display final telemetry message.
    }

    /*
     * Method to perform a relative move, based on encoder counts.
     * Encoders are not reset as the move is based on the current position.
     * Move will stop if any of three conditions occur:
     * 1) Move gets to the desired position
     * 2) Move runs out of time
     * 3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
            double leftInches, double rightInches,
            double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            // debug
            frontLeft.setTargetPosition(newLeftTarget);
            frontRight.setTargetPosition(newRightTarget);
            rearLeft.setTargetPosition(newLeftTarget);
            rearRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            rearLeft.setPower(Math.abs(speed));
            rearRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both
            // motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when
            // EITHER motor hits
            // its target position, the motion will stop. This is "safer" in the event that
            // the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the
            // robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), rearLeft.getCurrentPosition(),
                        rearRight.getCurrentPosition());
                updateLimelight(); // <== ADD THIS
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250); // optional pause after each move.
        }
    }

    public void shootOneBallAlways() {
        if (opModeIsActive()) {
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
    }

    public void rotateToColor(BallColor desired) {
        if (opModeIsActive()) {
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
            RedRobotTeleopMecanumFieldRelativeDrive.GenevaStatus status = getGenevaStatus(robot.feedingRotation);
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
            // while (opModeIsActive() && robot.feedingRotation.isBusy()) {
            // optional safety timeout
            // }

            robot.feedingRotation.setPower(0);
            robot.feedingRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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

    public void aimTurretAtBlueGoal() {
        if (opModeIsActive()) {
            LLResult result = robot.limelight.getLatestResult();

            if (result == null) {
                // No target → stop turret
                robot.turretSpinner.setPower(0);
                return;
            }

            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == 20) { // Red Alliance goal
                    double tx = tag.getTargetXDegrees();

                    // ------------------------
                    // Adaptive CR servo control
                    // ------------------------
                    double deadband = 0.5; // degrees, ignore tiny offsets
                    double maxPower = 0.15; // max speed at close range
                    double minPower = 0.05; // minimum speed to overcome friction
                    double kP = 0.05; // proportional gain

                    if (Math.abs(tx) < deadband) {
                        // close enough → stop
                        robot.turretSpinner.setPower(0);
                    } else {
                        // scale proportional to error, clamp to min/max
                        double scaledPower = Math.min(maxPower, Math.max(minPower, Math.abs(kP * tx)));
                        // apply direction
                        robot.turretSpinner.setPower(Math.signum(-tx) * scaledPower);
                    }

                    telemetry.addData("Turret Tracking", "Aiming at Tag 20");
                    telemetry.addData("tx", tx);
                    telemetry.addData("Power", robot.turretSpinner.getPower());
                    return;
                }
            }

            // No target found → stop turret
            robot.turretSpinner.setPower(0);
            telemetry.addData("Turret Tracking", "No target");
        }
    }

    private BallColor detectColor() {
        if (opModeIsActive()) {
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
        return null;
    }

    private BallColor detectColor1() {
        if (opModeIsActive()) {
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
        return null;
    }

    public void shootOneBall() {

        if (!opModeIsActive())
            return;

        // --------------------------
        // 1) Aim for 1.0 seconds
        // --------------------------
        double startTime = runtime.seconds();
        while (opModeIsActive() && runtime.seconds() - startTime < 2) {
            aimTurretAtBlueGoal();
            updateLimelightTelemetry();
            telemetry.update();
        }

        // --------------------------
        // 2) Spin flywheel up
        // --------------------------
        robot.launcher.setPower(0.85);
        startTime = runtime.seconds();
        while (opModeIsActive() && runtime.seconds() - startTime < 4) {
            aimTurretAtBlueGoal(); // keep aiming while spinning up
            updateLimelightTelemetry();
            telemetry.update();
        }

        // --------------------------
        // 3) Fire the shot
        // --------------------------
        safeKick();
        startTime = runtime.seconds();
        while (opModeIsActive() && runtime.seconds() - startTime < 4) {
            aimTurretAtBlueGoal();
            updateLimelightTelemetry();
            telemetry.update();
        }

        robot.kicker.setPosition(KICKER_DOWN);

        // --------------------------
        // 4) Brief pause between shots
        // --------------------------
        startTime = runtime.seconds();
        while (opModeIsActive() && runtime.seconds() - startTime < 4) {
            aimTurretAtBlueGoal();
            updateLimelightTelemetry();
            telemetry.update();
        }
    }

    public void macroRandomizedShoot() {
        if (opModeIsActive()) {
            aimTurretAtBlueGoal();
            try {
                Thread.sleep(750);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // restore interrupted status
            }
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
                        robot.feedingRotation.setPower(0);
                        break; // exit while loop
                    } else {
                        // Rotate feeder forward to find the target
                        robot.feedingRotation.setPower(1);
                    }
                }

                // Shoot the detected ball
                shootOneBall();
            }

            // Ensure feeder stops at the end
            robot.feedingRotation.setPower(0);
        }
    }

    public void macroSimpleShoot() {
        if (opModeIsActive()) {
            aimTurretAtBlueGoal();
            try {
                Thread.sleep(750);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // restore interrupted status
            }
            // Number of balls to shoot
            int ballsToShoot = 3;

            for (int i = 0; i < ballsToShoot; i++) {
                // Rotate until a ball of color GREEN or PURPLE is detected
                while (true) {
                    BallColor current = detectColor1();

                    if (current == BallColor.GREEN || current == BallColor.PURPLE) {
                        // Ball detected, stop feeder rotation
                        robot.feedingRotation.setPower(0);
                        break; // exit while loop
                    } else {
                        // Keep rotating forward to find the next ball
                        robot.feedingRotation.setPower(1);
                    }
                }

                // Shoot the detected ball
                shootOneBall();
            }

            // Ensure feeder stops at the end
            robot.feedingRotation.setPower(0);
        }
    }

    public void readAprilTagAndStoreOrder(int tagId) {
        if (opModeIsActive()) {
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
    }

    // Call this every loop
    private void updateFinColor() {
        if (opModeIsActive()) {
            // Read ONLY from the new color sensor
            BallColor detected = detectColor1();

            // Determine which fin is currently under the sensor
            RedRobotTeleopMecanumFieldRelativeDrive.GenevaStatus status = getGenevaStatus(robot.feedingRotation);
            int finIndex = status.fin; // 0, 1, or 2 depending on Geneva position

            // Update only that fin
            finColors[finIndex] = detected;

        }
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
        if (opModeIsActive()) {
            BallColor current1 = detectColor1();
            if (current1 == BallColor.GREEN || current1 == BallColor.PURPLE) {
                robot.kicker.setPosition(KICKER_UP);
            }
        }
    }

    public void feeding() {
        if (opModeIsActive()) {
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
                        robot.feedingRotation.setPower(feederUp ? 1 : -1);
                    } else {
                        robot.feedingRotation.setPower(0);
                        gamepad2.rumble(1, 1, 300); // same as before
                    }
                } else {
                    robot.feedingRotation.setPower(0);
                }

                return; // bypass auto layer
            }

            // -------------------------------------
            // LAYER 1: Auto (no LT held)
            // Pressing button ALWAYS starts spinning
            // AND starts a 1-second ignore period
            // -------------------------------------
            if (feederUp) {
                robot.feedingRotation.setPower(1);
                intakeColorIgnoreUntil = System.currentTimeMillis() + 1000; // 1 sec ignore
            }

            if (feederDown) {
                robot.feedingRotation.setPower(-1);
                intakeColorIgnoreUntil = System.currentTimeMillis() + 1000; // 1 sec ignore
            }

            // -------------------------------------
            // Auto-stop when color is detected
            // but ONLY after timeout expires
            // -------------------------------------
            if (System.currentTimeMillis() > intakeColorIgnoreUntil) {
                if (current1 == BallColor.GREEN || current1 == BallColor.PURPLE) {
                    robot.feedingRotation.setPower(0);
                }
            }
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

    public void updateTurretControl() {

        // ----------------------------
        // MANUAL MODE (bumper override)
        // ----------------------------
        if (gamepad2.left_bumper) {
            robot.turretSpinner.setPower(0.8); // rotate left
            telemetry.addData("Turret Mode", "Manual Left");
            return;
        }

        if (gamepad2.right_bumper) {
            robot.turretSpinner.setPower(-0.8); // rotate right
            telemetry.addData("Turret Mode", "Manual Right");
            return;
        }

        // -----------------------------------
        // AUTO MODE (no bumpers → auto aim)
        // -----------------------------------
        aimTurretAtBlueGoal(); // calls the auto-aim code
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

    public RedRobotTeleopMecanumFieldRelativeDrive.GenevaStatus getGenevaStatus(DcMotor feedingRotation) {
        if (opModeIsActive()) {
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

            return new RedRobotTeleopMecanumFieldRelativeDrive.GenevaStatus(zone, inGap);
        }
        return null;
    }

    // @Override
    public void onStop() {
        if (opModeIsActive()) {
            robot.limelight.stop();
        }
    }

    public void updateKicker() {
        if (opModeIsActive()) {

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
    }

    // Call this method whenever you want to update telemetry in Auto
    public void updateTelemetry() {
        if (opModeIsActive()) {
            // ----- Limelight -----
            LLResult result = robot.limelight.getLatestResult();
            if (result != null) {
                for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()) {
                    for (LLResultTypes.ColorResult colorResult : result.getColorResults()) {
                        telemetry.addData("Color", "X: " + JavaUtil.formatNumber(colorResult.getTargetXDegrees(), 2)
                                + ", Y: " + JavaUtil.formatNumber(colorResult.getTargetYDegrees(), 2));
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            // ----- Separate AprilTag detection for MOTIF -----
            if (aprilOrderSet) {
                telemetry.addData("AprilOrder",
                        "0: " + aprilOrder[0] + ", 1: " + aprilOrder[1] + ", 2: " + aprilOrder[2]);
            }

            // ----- Current ball color -----
            BallColor current1 = detectColor1();
            telemetry.addData("Current Ball Sensor1", current1); // shows NONE, GREEN, or PURPLE

            // ----- HSV from color sensor1 -----
            NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
            Color.colorToHSV(colors1.toColor(), hsvValues);

            // ----- Fin colors -----
            telemetry.addData("Fin Colors",
                    "0: " + finColors[0] + " 1: " + finColors[1] + " 2: " + finColors[2]);

            // ----- Optional distance -----
            if (colorSensor instanceof DistanceSensor) {
                double dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
                // telemetry.addData("Distance Sensor0 (cm)", "%.2f", dist);
            }
            if (colorSensor1 instanceof DistanceSensor) {
                double dist = ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM);
                // telemetry.addData("Distance Sensor1 (cm)", "%.2f", dist);
            }

            // ----- Color sensor gain -----
            // telemetry.addData("Color Sensor Gain", colorGain);

            telemetry.update(); // send all telemetry at once
        }
    }

    public void updateLimelightTelemetry() {
        if (opModeIsActive()) {
            if (robot.limelight == null)
                return; // safety

            LLStatus status = robot.limelight.getStatus();
            LLResult result = robot.limelight.getLatestResult();

            if (result != null) {
                Pose3D botpose = result.getBotpose();

                for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()) {
                    telemetry.addData("Fiducial",
                            "ID: " + fiducialResult.getFiducialId() +
                                    ", X: " + JavaUtil.formatNumber(fiducialResult.getTargetXDegrees(), 2) +
                                    ", Y: " + JavaUtil.formatNumber(fiducialResult.getTargetYDegrees(), 2));
                }

                for (LLResultTypes.ColorResult colorResult : result.getColorResults()) {
                    telemetry.addData("Color",
                            "X: " + JavaUtil.formatNumber(colorResult.getTargetXDegrees(), 2) +
                                    ", Y: " + JavaUtil.formatNumber(colorResult.getTargetYDegrees(), 2));
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
        }

    }

    public void updateLimelight() {// Limelight3A
        LLStatus status = robot.limelight.getStatus();
        // telemetry.addData("Name", status.getName());
        // telemetry.addData("LL", "Temp: " + JavaUtil.formatNumber(status.getTemp(), 1)
        // + "C, CPU: "
        // + JavaUtil.formatNumber(status.getCpu(), 1) + "%, FPS: " +
        // Math.round(status.getFps()));
        // telemetry.addData("Pipeline",
        // "Index: " + status.getPipelineIndex() + ", Type: " +
        // status.getPipelineType());
        LLResult result = robot.limelight.getLatestResult();
        if (result != null) {
            // Access general information.
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            // telemetry.addData("PythonOutput",
            // JavaUtil.makeTextFromList(result.getPythonOutput(), ","));
            // telemetry.addData("tx", result.getTx());
            // telemetry.addData("txnc", result.getTxNC());
            // telemetry.addData("ty", result.getTy());
            // telemetry.addData("tync", result.getTyNC());
            // telemetry.addData("Botpose", botpose.toString());
            // telemetry.addData("LL Latency", captureLatency + targetingLatency);
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

    }

}
