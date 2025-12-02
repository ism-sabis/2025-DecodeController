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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotTeleopMecanumFieldRelativeDrive;

import org.firstinspires.ftc.teamcode.BallColor;


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

@Autonomous(name = "Robot: Auto Drive By Encoder", group = "Robot")
@Disabled
public class RobotAutoDriveByEncoder_Linear extends LinearOpMode {
    final private org.firstinspires.ftc.teamcode.RobotHardware robot = new RobotHardware();
    /* Declare OpMode members. */
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Kicker auto-cycle state
    boolean kickerCycling = false;
    long kickerTimer = 0;

    // Kicker positions
    static final double KICKER_DOWN = 0.8;
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
    float colorGain = 2f;
    final float[] hsvValues = new float[3];
    boolean xPrev = false;

    BallColor[] aprilOrder = new BallColor[3];

    BallColor[] finColors = {
            BallColor.NONE, BallColor.NONE, BallColor.NONE};

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
    static final double DRIVE_GEAR_REDUCTION = 2.0; // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {



        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        rearLeft = hardwareMap.get(DcMotor.class, "rear_left");
        rearRight = hardwareMap.get(DcMotor.class, "rear_right");

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

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, 48, 48, 5.0); // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn Left 12 Inches with 4 Sec timeout
        shootOneBall();
        encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Forward 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // S3: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn Left 12 Inches with 4 Sec timeout
        macroRandomizedShoot();
        encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Forward 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // S3: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn Left 12 Inches with 4 Sec timeout
        macroRandomizedShoot();
        encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Forward 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // S3: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED, -12, 12, 4.0); // S2: Turn Left 12 Inches with 4 Sec timeout
        macroRandomizedShoot();
        encoderDrive(TURN_SPEED, 12, -12, 4.0); // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 24, 24, 4.0); // S3: Forward 24 Inches with 4 Sec timeout

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
        while (opModeIsActive() && robot.feedingRotation.isBusy()) {
            // optional safety timeout
        }

        robot.feedingRotation.setPower(0);
        robot.feedingRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public BallColor detectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        int r = (int) (colors.red * 255);
        int b = (int) (colors.blue * 255);

        if (r > b + 40)
            return BallColor.RED;
        if (b > r + 40)
            return BallColor.BLUE;
        return BallColor.NONE;
    }

    public void shootOneBall() {
        // Spin flywheel up
        robot.launcher.setPower(1.0);
        sleep(300); // change to your real spin-up time

        // Fire
        robot.kicker.setPosition(KICKER_UP);
        sleep(130);
        robot.kicker.setPosition(KICKER_DOWN);

        // ===============================
        // TUNEABLE PAUSE BETWEEN SHOTS
        // ===============================
        sleep(200); // <---- adjust this value
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
            case 3:
                aprilOrder[0] = BallColor.BLUE;
                aprilOrder[1] = BallColor.RED;
                aprilOrder[2] = BallColor.BLUE;
                break;

            case 7:
                aprilOrder[0] = BallColor.RED;
                aprilOrder[1] = BallColor.BLUE;
                aprilOrder[2] = BallColor.RED;
                break;

            default:
                aprilOrder[0] = BallColor.NONE;
                aprilOrder[1] = BallColor.NONE;
                aprilOrder[2] = BallColor.NONE;
                break;
        }
    }

    public void updateFinColors(GenevaStatus status) {

        BallColor current = detectColor();

        // Detect rising edge (a new ball just appeared at the sensor)
        if (current != BallColor.NONE && lastDetected == BallColor.NONE) {

            // Assign this new ball to the fin currently at the intake
            finColors[status.fin] = current;
        }

        lastDetected = current;

        // Telemetry
        telemetry.addData("Fin Colors",
                "0:" + finColors[0] + "  1:" + finColors[1] + "  2:" + finColors[2]);
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

    public void updateKicker() {

        // Check launcher speed
        boolean launcherAtSpeed = robot.launcher.getPower() >= LAUNCHER_MIN_POWER;

        // Start cycle only if:
        // - Button pressed (tap)
        // - Not already cycling
        // - Launcher is up to speed
        if (!kickerCycling && launcherAtSpeed) {
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

    //@Override
    //public void stop() {
    //    robot.limelight.stop();
    //}
}
