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
import com.qualcomm.robotcore.hardware.AnalogInput;
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

@Autonomous(name = "Red Audience Robot: Auto Drive By Encoder", group = "Robot")
// @Disabled
public class RedAudienceRobotAutoDriveByEncoder_Linear extends LinearOpMode {
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

    // ========== INDEXER TRACKING ==========
    static final int NUM_SLOTS = 3;
    BallColor[] indexerSlots = {BallColor.UNIDENTIFIED, BallColor.UNIDENTIFIED, BallColor.UNIDENTIFIED};
    static final double ANGLE_PER_SLOT = 120.0;
    private int currentPosition = 0;
    private boolean indexerMoving = false;
    private long intakeStopTime = 0;
    private boolean intakeDelayUsed = false;

    private RTPAxon servo;
    private RTPAxon servo1;
    private AnalogInput encoder;
    private AnalogInput encoder1;
    private CRServo crservo;
    private CRServo crservo1;

    // Nonblocking shoot state machine
    private boolean shootAllActive = false;
    private int shootAllRemaining = 0;
    private long shootAllStateStartMs = 0;
    private enum ShootAllState { IDLE, CHECK_SLOT, START_SHOOT, WAIT_SHOOT, NEXT_MOVE, WAIT_REACH, DONE }
    private ShootAllState shootAllState = ShootAllState.IDLE;

    private LauncherState launcherState = LauncherState.IDLE;

    // Distance and power calculation
    private double distanceNew = 0;

    // Kicker auto-cycle state
    boolean kickerCycling = false;
    long kickerTimer = 0;

    // Kicker positions
    static final double KICKER_DOWN = 0.725;
    static final double KICKER_UP = 0.5; // adjust if needed

    // Timing
    static final long KICK_TIME = 500; // milliseconds for kick

    // Launcher speed threshold (adjust)
    static final double LAUNCHER_MIN_POWER = 0.05; // normalized 0 to 1.0

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

    BallColor[] aprilOrder = {BallColor.NONE, BallColor.NONE, BallColor.NONE};

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

    // Nonblocking shoot pattern state machine
    private boolean shootPatternActive = false;
    private int shootPatternIndex = 0;
    private long shootPatternStateStartMs = 0;
    private enum ShootPatternState { IDLE, MOVE, WAIT_REACH, START_SHOOT, WAIT_SHOOT, NEXT, DONE }
    private ShootPatternState shootPatternState = ShootPatternState.IDLE;

    // Nonblocking single-shot by color
    private boolean shootOneActive = false;
    private BallColor shootOneTarget = BallColor.NONE;
    private long shootOneStartMs = 0;
    private enum ShootOneState { IDLE, MOVE, WAIT_REACH, START_SHOOT, WAIT_SHOOT, DONE }
    private ShootOneState shootOneState = ShootOneState.IDLE;

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

        // Initialize indexer servos
        crservo = hardwareMap.crservo.get("indexer");
        crservo1 = hardwareMap.crservo.get("indexer1");
        encoder = hardwareMap.get(AnalogInput.class, "indexerEncoder");
        encoder1 = hardwareMap.get(AnalogInput.class, "indexerEncoder1");
        servo = new RTPAxon(crservo, encoder);
        servo1 = new RTPAxon(crservo1, encoder1);
        servo.setDirectionChangeCompensation(11);
        servo1.setRtp(false);

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

        // Limelight3A
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

        // ----- Separate AprilTag detection for MOTIF -----
        if (!aprilOrderSet) {
            LLResult tagResult = robot.limelight.getLatestResult();
            if (tagResult != null) {
                List<LLResultTypes.FiducialResult> tags = tagResult.getFiducialResults();
                if (!tags.isEmpty()) {
                    int detectedTagId = tags.get(0).getFiducialId();

                    // Store the order for this match
                    readAprilTagAndStoreOrder(detectedTagId);
                    aprilOrderSet = true; // lock order

                    // Telemetry: show the scanned order
                    telemetry.addData("AprilTag ID", detectedTagId);
                    telemetry.addData("AprilOrder",
                            "0: " + aprilOrder[0] + ", 1: " + aprilOrder[1] + ", 2: " + aprilOrder[2]);
                    telemetry.update();
                }
            }
        }

        BallColor current1 = detectColor1();
        telemetry.addData("Current Ball Sensor1", current1); // shows NONE, GREEN, or PURPLE

        NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();

        // Convert to HSV
        Color.colorToHSV(colors1.toColor(), hsvValues);

        // telemetry.addData("Current Ball", current); // shows NONE, GREEN, or PURPLE
        telemetry.addData("Fin Colors",
                "0: " + finColors[0] + " 1: " + finColors[1] + " 2: " + finColors[2]);

        // Show distance if supported
        if (colorSensor instanceof DistanceSensor) {
            double dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            // telemetry.addData("Distance (cm)", "%.2f", dist);
        }
        // Show distance if supported
        if (colorSensor1 instanceof DistanceSensor) {
            double dist = ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM);
            // telemetry.addData("Distance (cm)", "%.2f", dist);
        }

        // Apply gain to the color sensor
        colorSensor.setGain(colorGain);
        colorSensor1.setGain(colorGain);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Start in Quadrant 4

        encoderDrive(DRIVE_SPEED, 7, 7, 2); // S1: Forward toward Y-axis (mirrored), 5 sec timeout

        encoderDrive(TURN_SPEED, -23, 23, 4.0); // S2: Turn left 50 inches (mirrored right), 4 sec timeout

        encoderDrive(DRIVE_SPEED, 6.5, 6.5, 4.0); // S3: Forward 24 inches (mirrored), 4 sec timeout

        encoderDrive(TURN_SPEED, 23, -23, 4.0); // S2: Turn right 12 inches (mirrored left), 4 sec timeout

        autoShootAllBalls(); // Shoot all preloaded balls

        encoderDrive(DRIVE_SPEED, 10, 10, 4.0); // S3: Reverse 24 inches (mirrored), 4 sec timeout


        encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn right 12 inches (mirrored left), 4 sec timeout

        displayAprilTagOrder();
        runShootInPatternBlocking(); // Shoot balls based on AprilTag order
          
          
        encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn left 12 inches (mirrored right), 4 sec timeout
          
          
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // S3: Forward 24 inches (mirrored), 4 sec timeout
          
          
        encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn right 12 inches (mirrored left), 4 sec timeout
          
          
        encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Reverse 24 inches (mirrored), 4 sec timeout
          
          
        encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn right 12 inches (mirrored left), 4 sec timeout
          
          
        encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Reverse 24 inches (mirrored), 4 sec timeout
          
          
        encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn right 12 inches (mirrored left), 4 sec timeout
          
          
        runShootInPatternBlocking(); // Shoot balls based on AprilTag order
          
          
        encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn left 12 inches (mirrored right), 4 sec timeout
          
          
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // S3: Forward 24 inches (mirrored), 4 sec timeout
          
          
        encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn right 12 inches (mirrored left), 4 sec timeout
          
          
        encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Reverse 24 inches (mirrored), 4 sec timeout
          
          
        encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn right 12 inches (mirrored left), 4 sec timeout
          
         
        encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Reverse 24 inches (mirrored), 4 sec timeout
          
          
        encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn right 12 inches (mirrored left), 4 sec timeout
          
          
        runShootInPatternBlocking(); // Shoot balls based on AprilTag order
          
          
        encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn left 12 inches (mirrored right), 4 sec timeout
          
          
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // S3: Forward 24 inches (mirrored), 4 sec timeout

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

    // ============================================
    // INDEXER & BALL TRACKING HELPERS
    // ============================================

    int getSlotAtShootingPosition() {
        int[] shooterSlots = {0, 2, 1};
        return shooterSlots[currentPosition / 2];
    }

    boolean isAtSensorPosition() {
        return currentPosition % 2 == 0;
    }

    void applyPositionDelta(double degreesMoved) {
        int steps = (int) Math.round(degreesMoved / 60.0);
        currentPosition = (currentPosition + steps + 6) % 6;
        if (currentPosition < 0) currentPosition += 6;
    }

    void commandIndexerRotation(double degreesToMove) {
        indexerMoving = true;
        intakeDelayUsed = false;
        servo.changeTargetRotation(degreesToMove);
        servo1.changeTargetRotation(degreesToMove);
        applyPositionDelta(degreesToMove);
    }

    void rotateIndexerTo(int targetIdx) {
        if (targetIdx < 0 || targetIdx >= NUM_SLOTS) return;
        int currentSlot = getSlotAtShootingPosition();
        int[] slotToSeq = {0, 2, 1};
        int currentSeq = slotToSeq[currentSlot];
        int targetSeq = slotToSeq[targetIdx];
        int forwardDelta = (targetSeq - currentSeq + NUM_SLOTS) % NUM_SLOTS;
        int backwardDelta = (currentSeq - targetSeq + NUM_SLOTS) % NUM_SLOTS;
        double degreesToMove;
        if (forwardDelta <= backwardDelta) {
            degreesToMove = forwardDelta * ANGLE_PER_SLOT;
        } else {
            degreesToMove = -backwardDelta * ANGLE_PER_SLOT;
        }
        commandIndexerRotation(degreesToMove);
    }

    void rotateIndexerToIntake(int targetIdx) {
        if (targetIdx < 0 || targetIdx >= NUM_SLOTS) return;
        int currentSlot = getSlotAtShootingPosition();
        int[] slotToSeq = {0, 2, 1};
        int currentSeq = slotToSeq[currentSlot];
        int targetSeq = slotToSeq[targetIdx];
        int forwardDelta = (targetSeq - currentSeq + NUM_SLOTS) % NUM_SLOTS;
        int backwardDelta = (currentSeq - targetSeq + NUM_SLOTS) % NUM_SLOTS;
        double degreesToMove;
        if (forwardDelta <= backwardDelta) {
            degreesToMove = forwardDelta * ANGLE_PER_SLOT;
        } else {
            degreesToMove = -backwardDelta * ANGLE_PER_SLOT;
        }
        degreesToMove += 180.0;
        if (degreesToMove > 180.0) {
            degreesToMove -= 360.0;
        } else if (degreesToMove < -180.0) {
            degreesToMove += 360.0;
        }
        commandIndexerRotation(degreesToMove);
    }

    int findFirstEmptySlot() {
        for (int i = 0; i < NUM_SLOTS; i++) {
            if (indexerSlots[i] != BallColor.GREEN && indexerSlots[i] != BallColor.PURPLE) {
                return i;
            }
        }
        return -1;
    }

    int findNearestSlotWithColor(BallColor target) {
        int currentSlot = getSlotAtShootingPosition();
        int bestDist = NUM_SLOTS;
        int bestIdx = -1;
        for (int i = 0; i < NUM_SLOTS; i++) {
            if (indexerSlots[i] == target) {
                int forward = (i - currentSlot + NUM_SLOTS) % NUM_SLOTS;
                int backward = (currentSlot - i + NUM_SLOTS) % NUM_SLOTS;
                int d = Math.min(forward, backward);
                if (d < bestDist) {
                    bestDist = d;
                    bestIdx = i;
                }
            }
        }
        return bestIdx;
    }

    boolean isIndexerAtTarget(double tolerance) {
        return servo.isAtTarget(tolerance) && servo1.isAtTarget(tolerance);
    }

    void updateShooterPosColor() {
        BallColor detected = detectColor1();
        if (!isAtSensorPosition() || indexerMoving) return;
        int shootingSlot = getSlotAtShootingPosition();
        BallColor current = indexerSlots[shootingSlot];
        if (detected == BallColor.NONE) {
            if (current == BallColor.UNIDENTIFIED) {
                indexerSlots[shootingSlot] = BallColor.NONE;
            }
            return;
        }
        if (current == BallColor.NONE || current == BallColor.UNIDENTIFIED) {
            indexerSlots[shootingSlot] = detected;
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

    public void aimTurretAtRedGoal() {
        if (opModeIsActive()) {
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

                    telemetry.addData("Turret Tracking", "Aiming at Tag 24");
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
            aimTurretAtRedGoal();
            updateLimelightTelemetry();
            telemetry.update();
        }

        // --------------------------
        // 2) Spin flywheel up
        // --------------------------
        robot.launcher.setPower(0.075);
        startTime = runtime.seconds();
        while (opModeIsActive() && runtime.seconds() - startTime < 4) {
            aimTurretAtRedGoal(); // keep aiming while spinning up
            updateLimelightTelemetry();
            telemetry.update();
        }

        // --------------------------
        // 3) Fire the shot
        // --------------------------
        safeKick();
        startTime = runtime.seconds();
        while (opModeIsActive() && runtime.seconds() - startTime < 4) {
            aimTurretAtRedGoal();
            updateLimelightTelemetry();
            telemetry.update();
        }

        robot.kicker.setPosition(KICKER_DOWN);

        // --------------------------
        // 4) Brief pause between shots
        // --------------------------
        startTime = runtime.seconds();
        while (opModeIsActive() && runtime.seconds() - startTime < 4) {
            aimTurretAtRedGoal();
            updateLimelightTelemetry();
            telemetry.update();
        }
    }

    public void macroRandomizedShoot() {
        if (opModeIsActive()) {
            aimTurretAtRedGoal();
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
            aimTurretAtRedGoal();
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

    // ============================================
    // AUTONOMOUS SHOOT & INTAKE MACROS
    // ============================================

    /**
     * Autonomous intake one ball - rotate to empty slot at intake position and run intake.
     */
    public void autoIntakeOneBall() {
        int emptySlot = findFirstEmptySlot();
        if (emptySlot == -1) {
            telemetry.addLine("AUTO: Indexer full!");
            telemetry.update();
            return;
        }
        robot.feedingRotation.setPower(1.0);
        rotateIndexerToIntake(emptySlot);
        
        // Wait for indexer to reach target
        long timeoutMs = System.currentTimeMillis() + 3000;
        while (opModeIsActive() && !isIndexerAtTarget(5) && System.currentTimeMillis() < timeoutMs) {
            servo.update();
            servo1.setRawPower(servo.getPower() * 0.5);
        }
    }

    /**
     * Autonomous shoot all balls - rotates through slots and shoots any GREEN/PURPLE balls.
     */
    public void autoShootAllBalls() {
        shootAllActive = true;
        shootAllRemaining = NUM_SLOTS;
        robot.feedingRotation.setPower(1.0);
        shootAllState = ShootAllState.CHECK_SLOT;
        shootAllStateStartMs = System.currentTimeMillis();
        
        while (opModeIsActive() && shootAllActive) {
            servo.update();
            servo1.setRawPower(servo.getPower() * 0.5);
            updateShooterPosColor();
            updateAutoShootAll();
            updateLauncherAuto();
        }
    }

    /**
     * Update method for nonblocking shoot-all in autonomous.
     */
    private void updateAutoShootAll() {
        if (!shootAllActive) return;
        
        // Check if indexer movement complete
        if (indexerMoving && (servo.isAtTarget(40) || Math.abs(servo.getPower()) < 0.05)) {
            indexerMoving = false;
            if (!intakeDelayUsed) {
                intakeStopTime = System.currentTimeMillis() + 250;
                intakeDelayUsed = true;
            } else {
                intakeStopTime = 0;
            }
        }
        
        // Stop intake after delay
        if (!indexerMoving && intakeStopTime > 0 && System.currentTimeMillis() > intakeStopTime) {
            robot.feedingRotation.setPower(0);
        }
        
        switch (shootAllState) {
            case CHECK_SLOT: {
                // Always shoot current slot regardless of color
                if (launcherState == LauncherState.IDLE) {
                    launcherState = LauncherState.STARTING;
                    shootAllState = ShootAllState.WAIT_SHOOT;
                }
                break;
            }
            case WAIT_SHOOT: {
                if (launcherState == LauncherState.IDLE) {
                    int shootingSlot = getSlotAtShootingPosition();
                    indexerSlots[shootingSlot] = BallColor.NONE;
                    shootAllRemaining -= 1;
                    if (shootAllRemaining <= 0) {
                        shootAllState = ShootAllState.DONE;
                    } else {
                        int nextSlot = (shootingSlot + 1) % NUM_SLOTS;
                        rotateIndexerTo(nextSlot);
                        shootAllState = ShootAllState.WAIT_REACH;
                        shootAllStateStartMs = System.currentTimeMillis();
                    }
                }
                break;
            }
            case WAIT_REACH: {
                boolean reached = isIndexerAtTarget(5);
                boolean timedOut = System.currentTimeMillis() - shootAllStateStartMs >= 1500;
                if (reached || timedOut) {
                    shootAllState = ShootAllState.CHECK_SLOT;
                }
                break;
            }
            case DONE: {
                robot.feedingRotation.setPower(0);
                shootAllActive = false;
                shootAllState = ShootAllState.IDLE;
                break;
            }
            default:
                break;
        }
    }

    /**
     * Update launcher state machine during autonomous shoot.
     */
    private void updateLauncherAuto() {
        if (launcherState == LauncherState.STARTING) {
            double power = calculateLauncherPower();
            robot.launcher.setPower(power);
            runtime.reset();
            launcherState = LauncherState.SPINNING;
        } else if (launcherState == LauncherState.SPINNING && runtime.seconds() >= calculateSpinUpTime()) {
            robot.kicker.setPosition(KICKER_UP);
            runtime.reset();
            launcherState = LauncherState.KICKING;
        } else if (launcherState == LauncherState.KICKING && runtime.seconds() >= 0.5) {
            robot.kicker.setPosition(KICKER_DOWN);
            launcherState = LauncherState.UNKICKING;
        } else if (launcherState == LauncherState.UNKICKING) {
            robot.launcher.setPower(0);
            launcherState = LauncherState.IDLE;
        }
    }

    // ============================================
    // TURRET TRACKING
    // ============================================

    /**
     * Autonomous turret tracking - aims at red goal (tag 24) and waits until aligned.
     * Blocks until aligned or timeout.
     */
    public void autoAimTurret(long timeoutMs) {
        long startTime = System.currentTimeMillis();
        
        while (opModeIsActive() && System.currentTimeMillis() - startTime < timeoutMs) {
            LLResult result = robot.limelight.getLatestResult();
            
            if (result != null) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    if (tag.getFiducialId() == 24) { // Red Alliance goal
                        double tx = tag.getTargetXDegrees();
                        
                        double deadband = 1.0;
                        double maxPower = 0.17;
                        double minPower = 0.1;
                        double kP = 0.04;
                        
                        if (Math.abs(tx) < deadband) {
                            // Aligned - stop turret
                            robot.turretSpinner.setPower(0);
                            telemetry.addData("Turret", "Aligned with goal");
                            telemetry.update();
                            return; // Exit when aligned
                        } else {
                            // Scale proportional to error, clamp to min/max
                            double scaledPower = Math.abs(kP * tx);
                            if (scaledPower < minPower) {
                                scaledPower = minPower;
                            } else if (scaledPower > maxPower) {
                                scaledPower = maxPower;
                            }
                            robot.turretSpinner.setPower(Math.signum(tx) * scaledPower);
                            telemetry.addData("Turret", "Aiming... tx: " + String.format("%.2f", tx));
                            telemetry.update();
                        }
                        return; // Found tag, exit loop
                    }
                }
            }
            
            // No tag found - stop turret
            robot.turretSpinner.setPower(0);
            telemetry.addData("Turret", "No target found");
            telemetry.update();
        }
        
        // Timeout - stop turret
        robot.turretSpinner.setPower(0);
        telemetry.addData("Turret", "Timeout - stopped");
        telemetry.update();
    }

    /**
     * Continuous turret tracking - keeps aiming at goal while other tasks run.
     * Call this periodically during autonomous routines.
     */
    public void updateTurretAim() {
        LLResult result = robot.limelight.getLatestResult();
        
        if (result != null) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == 24) { // Red Alliance goal
                    double tx = tag.getTargetXDegrees();
                    
                    double deadband = 1.0;
                    double maxPower = 0.17;
                    double minPower = 0.1;
                    double kP = 0.04;
                    
                    if (Math.abs(tx) < deadband) {
                        robot.turretSpinner.setPower(0);
                    } else {
                        double scaledPower = Math.abs(kP * tx);
                        if (scaledPower < minPower) {
                            scaledPower = minPower;
                        } else if (scaledPower > maxPower) {
                            scaledPower = maxPower;
                        }
                        robot.turretSpinner.setPower(Math.signum(tx) * scaledPower);
                    }
                    return;
                }
            }
        }
        
        // No target - stop
        robot.turretSpinner.setPower(0);
    }

    // ============================================
    // LAUNCHER POWER CALCULATION
    // ============================================

    /**
     * Calculate distance from AprilTag using Limelight ta (target area).
     */
    public double getDistanceFromTag(double ta) {
        distanceNew = 72.34359 * Math.pow(ta, -0.479834);
        return distanceNew;
    }

    /**
     * Calculate launcher power based on distance to goal.
     */
    double calculateLauncherPower() {
        LLResult result = robot.limelight.getLatestResult();
        if (result == null) return 1;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() != 24) continue;

            double area = result.getTa();
            double launcherPower = (0.00303584 * distanceNew) + 0.586525;
            return launcherPower;
        }
        return 1;
    }

    /**
     * Calculate RPM needed for launcher.
     */
    double calculateRPM() {
        return 6000 * calculateLauncherPower();
    }

    /**
     * Calculate spin-up time for launcher.
     */
    double calculateSpinUpTime() {
        return 0.00000275688 * Math.pow(calculateRPM(), 1.54859);
    }

    /**
     * Autonomous shoot one ball - aims turret, spins up launcher with calculated power, and fires.
     */
    public void autoShootOneBall() {
        // 1. Aim turret at goal
        autoAimTurret(3000);
        
        // 2. Get distance from Limelight and calculate power
        LLResult result = robot.limelight.getLatestResult();
        if (result != null) {
            double ta = result.getTa();
            getDistanceFromTag(ta);
        }
        
        // 3. Spin up launcher with calculated power
        double power = calculateLauncherPower();
        robot.launcher.setPower(power);
        
        // 4. Wait for spin-up
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < calculateSpinUpTime()) {
            // Keep turret aimed during spin-up
            updateTurretAim();
        }
        
        // 5. Fire the kicker
        robot.kicker.setPosition(KICKER_UP);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5) {
            // Wait for kick
        }
        
        // 6. Reset kicker and stop launcher
        robot.kicker.setPosition(KICKER_DOWN);
        robot.launcher.setPower(0);
        telemetry.addData("Shot", "Complete - Distance: " + String.format("%.2f", distanceNew) + "in");
        telemetry.update();
    }

    // ============================================
    // APRIL TAG DISPLAY & SHOOT PATTERN MACRO
    // ============================================

    /**
     * Display the detected April tag order and shoot order on telemetry.
     * Called during autonomous init and loop to show current state.
     */
    private void displayAprilTagOrder() {
        if (!aprilOrderSet) {
            telemetry.addData("April Tag", "Scanning...");
        } else {
            telemetry.addData("Shoot Order", 
                String.format("%s | %s | %s", aprilOrder[0], aprilOrder[1], aprilOrder[2]));
        }
    }

    /**
     * Start the nonblocking shoot pattern sequence.
     * Shoots balls in the order specified by aprilOrder (from detected tag).
     * Skips NONE entries automatically.
     */
    void startShootInPattern() {
        if (shootPatternActive) return;
        shootPatternActive = true;
        shootPatternIndex = 0;
        robot.feedingRotation.setPower(1.0);
        shootPatternState = ShootPatternState.NEXT;
    }
    
    /**
     * LinearOpMode helper: Run shoot in pattern and wait for completion.
     * Continuously updates servo and state machines until done.
     */
    void runShootInPatternBlocking() {
        startShootInPattern();
        while (opModeIsActive() && shootPatternActive) {
            servo.update();
            servo1.setRawPower(servo.getPower() * 0.5);
            updateShooterPosColor();
            updateShootInPattern();
            updateLauncherAuto();
            
            // Handle indexer stop logic
            if (indexerMoving && (servo.isAtTarget(40) || Math.abs(servo.getPower()) < 0.05)) {
                indexerMoving = false;
                if (!intakeDelayUsed) {
                    intakeStopTime = System.currentTimeMillis() + 250;
                    intakeDelayUsed = true;
                } else {
                    intakeStopTime = 0;
                }
            }
            
            // Stop intake after delay
            if (!indexerMoving && intakeStopTime > 0 && System.currentTimeMillis() > intakeStopTime) {
                robot.feedingRotation.setPower(0);
            }
            
            telemetry.addData("Pattern", "Shooting... Index: " + shootPatternIndex);
            telemetry.update();
        }
    }

    /**
     * Update the nonblocking shoot pattern state machine.
     * Call this in the loop to execute the pattern shooting sequence.
     */
    void updateShootInPattern() {
        if (!shootPatternActive) return;
        
        // Keep intake running while indexer moving
        if (indexerMoving) {
            robot.feedingRotation.setPower(1.0);
            intakeStopTime = 0;
        }
        
        // Wait for any active shootOne to complete
        if (shootOneActive) {
            updateShootOneBall();
            return;
        }
        
        // Move to next color in pattern
        if (shootPatternIndex >= NUM_SLOTS) {
            // Done with all colors
            robot.feedingRotation.setPower(0);
            shootPatternActive = false;
            return;
        }
        
        BallColor toShoot = aprilOrder[shootPatternIndex];
        shootPatternIndex++;
        
        if (toShoot == BallColor.NONE) {
            // Skip NONE entries
            return;
        }
        
        // Find and shoot this color
        int idx = findNearestSlotWithColor(toShoot);
        if (idx == -1) {
            return; // Color not found, skip
        }
        
        // Start shooting this ball
        shootOneActive = true;
        shootOneTarget = toShoot;
        if (!shootPatternActive) {
            robot.feedingRotation.setPower(0);
        }
        rotateIndexerTo(idx);
        shootOneState = ShootOneState.WAIT_REACH;
        shootOneStartMs = System.currentTimeMillis();
    }

    /**
     * Update shoot one ball state machine for autonomous.
     */
    void updateShootOneBall() {
        if (!shootOneActive) return;
        switch (shootOneState) {
            case WAIT_REACH: {
                boolean reached = isIndexerAtTarget(5);
                boolean timedOut = System.currentTimeMillis() - shootOneStartMs >= 1500;
                if (reached || timedOut) {
                    shootOneState = ShootOneState.START_SHOOT;
                }
                break;
            }
            case START_SHOOT: {
                if (launcherState == LauncherState.IDLE) {
                    launcherState = LauncherState.STARTING;
                    shootOneState = ShootOneState.WAIT_SHOOT;
                }
                break;
            }
            case WAIT_SHOOT: {
                if (launcherState == LauncherState.IDLE) {
                    int shootingSlot = getSlotAtShootingPosition();
                    indexerSlots[shootingSlot] = BallColor.NONE;
                    shootOneState = ShootOneState.DONE;
                }
                break;
            }
            case DONE: {
                if (!shootPatternActive) {
                    robot.feedingRotation.setPower(0);
                }
                shootOneActive = false;
                shootOneState = ShootOneState.IDLE;
                break;
            }
            default:
                break;
        }
    }

}
