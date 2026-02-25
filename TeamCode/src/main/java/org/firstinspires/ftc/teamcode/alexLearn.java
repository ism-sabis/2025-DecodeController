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

@Autonomous(name = "alexLearn: Auto Drive By Encoder", group = "Robot")
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

 

    // public IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

 
 
    // Slow mode
    private double maxSpeed = 1.0; // default full speed

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
    public void runOpMode() throws InterruptedException {

        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

      

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

        // Ensure all motors are stopped during init
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        robot.feedingRotation.setPower(0);
        robot.launcher.setPower(0);
        robot.leftLift.setPower(0);
        robot.rightLift.setPower(0);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                rearLeft.getCurrentPosition(),
                rearRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

      
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
     * Autonomous reindex - rotate through all slots and identify ball colors.
     * Must be called before autoShootAllBalls() to populate indexerSlots[].
     */
    public void autoReindexIdentifyColors() {
        robot.feedingRotation.setPower(1.0);
        
        // Rotate through each slot and read color
        for (int i = 0; i < NUM_SLOTS; i++) {
            int targetSlot = i;
            rotateIndexerTo(targetSlot);
            
            // Wait for indexer to reach target
            long timeoutMs = System.currentTimeMillis() + 2000;
            while (opModeIsActive() && !isIndexerAtTarget(5) && System.currentTimeMillis() < timeoutMs) {
                servo.update();
                servo1.setRawPower(servo.getPower() * 0.5);
                updateShooterPosColor();  // Read color while rotating
            }
            
            // Allow sensor to stabilize and sample again
            long stabilizeMs = System.currentTimeMillis() + 300;
            while (opModeIsActive() && System.currentTimeMillis() < stabilizeMs) {
                servo.update();
                servo1.setRawPower(servo.getPower() * 0.5);
                updateShooterPosColor();  // Keep sampling during stabilize time
            }
        }
        
        // Ensure indexer is completely stopped before returning
        robot.feedingRotation.setPower(0);
        servo.setPower(0);
        servo1.setPower(0);
        
        // Wait a bit for everything to settle
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        
        // Reset state flags to ensure shooting doesn't interfere
        shootAllActive = false;
        launcherState = LauncherState.IDLE;
        indexerMoving = false;
    }

    /**
     * Autonomous shoot all balls - rotates through slots and shoots any GREEN/PURPLE balls.
     */
    public void autoShootAllBalls() {
        // Ensure clean start - no leftover movement from reindex
        indexerMoving = false;
        shootAllActive = true;
        shootAllRemaining = NUM_SLOTS;
        launcherState = LauncherState.IDLE;
        robot.feedingRotation.setPower(1.0);  // Keep intake running full power
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
        
        // Always keep intake running during entire shoot-all sequence
        robot.feedingRotation.setPower(1.0);
        
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
        // Continuously update distance from Limelight
        LLResult result = robot.limelight.getLatestResult();
        if (result != null) {
            double ta = result.getTa();
            getDistanceFromTag(ta);
        }
        
        if (launcherState == LauncherState.STARTING) {
            double power = calculateLauncherPower();
            robot.launcher.setPower(power);
            runtime.reset();
            launcherState = LauncherState.SPINNING;
        } else if (launcherState == LauncherState.SPINNING) {
            double spinUpTime = calculateSpinUpTime();
            if (runtime.seconds() >= spinUpTime) {
                robot.kicker.setPosition(KICKER_UP);
                runtime.reset();
                launcherState = LauncherState.KICKING;
            }
        } else if (launcherState == LauncherState.KICKING) {
            if (runtime.seconds() >= 1.0) {
                robot.kicker.setPosition(KICKER_DOWN);
                runtime.reset();
                launcherState = LauncherState.UNKICKING;
            }
        } else if (launcherState == LauncherState.UNKICKING) {
            if (runtime.seconds() >= 0.3) {
                robot.launcher.setPower(0);
                launcherState = LauncherState.IDLE;
            }
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
                        
                        double deadband = 2.0;
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
                    
                    double deadband = 2.0;
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

            double ta = result.getTa();
            // Calculate distance from target area
            double distance = 72.34359 * Math.pow(ta, -0.479834);
            distanceNew = distance;  // Update for telemetry
            
            double launcherPower = (0.00303584 * distance) + 0.586525;
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
        // 1. Aim turret at goal (increased time)
        autoAimTurret(7000);
        
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
