package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RTPAxon {
    // Encoder for servo position feedback
    private final AnalogInput servoEncoder;
    // Continuous rotation servo
    private final CRServo servo;
    // Run-to-position mode flag
    private boolean rtp;
    // Current power applied to servo
    private double power;
    // Maximum allowed power
    private double maxPower;
    // Direction of servo movement
    private Direction direction;
    // Last measured angle
    private double previousAngle;
    // Accumulated rotation in degrees
    private double totalRotation;
    // Filtered total rotation (low-pass filtered)
    private double filteredTotalRotation;
    // Target rotation in degrees
    private double targetRotation;
    // Last change direction (positive or negative)
    private double lastChangeDirection = 0;
    // Compensation offset when switching directions
    private double directionChangeCompensation = 0;

    // PID controller coefficients and state
    private double kP;
    private double kI;
    private double kD;
    private double integralSum;
    private double lastError;
    private double maxIntegralSum;
    private ElapsedTime pidTimer;

    // Initialization and debug fields
    public double STARTPOS;
    public int ntry = 0;
    public int cliffs = 0;
    public double homeAngle;

    // Direction enum for servo
    public enum Direction {
        FORWARD,
        REVERSE
    }

    // region constructors

    // Basic constructor, defaults to FORWARD direction
    public RTPAxon(CRServo servo, AnalogInput encoder) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        direction = Direction.FORWARD;
        initialize();
    }

    // Constructor with explicit direction
    public RTPAxon(CRServo servo, AnalogInput encoder, Direction direction) {
        this(servo, encoder);
        this.direction = direction;
        initialize();
    }

    // Initialization logic for servo and encoder
    private void initialize() {
        servo.setPower(0);
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {
        }

        // Try to get a valid starting position
        do {
            STARTPOS = getCurrentAngle();
            if (Math.abs(STARTPOS) > 1) {
                previousAngle = getCurrentAngle();
            } else {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException ignored) {
                }
            }
            ntry++;
        } while (Math.abs(previousAngle) < 0.2 && (ntry < 50));

        totalRotation = 0;
        filteredTotalRotation = 0;
        homeAngle = previousAngle;

        // Default PID coefficients
        kP = 0.017;
        kI = 0.002;
        kD = 0.000;
        integralSum = 0.0;
        lastError = 0.0;
        maxIntegralSum = 100.0;
        pidTimer = new ElapsedTime();
        pidTimer.reset();

        maxPower = 0.25;
        cliffs = 0;

    }
    // endregion

    // Set servo direction
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    // Set power to servo, respecting direction and maxPower
    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    public void setRawPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    // Get current power
    public double getPower() {
        return power;
    }

    // Set maximum allowed power
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    // Get maximum allowed power
    public double getMaxPower() {
        return maxPower;
    }

    // Enable or disable run-to-position mode
    public void setRtp(boolean rtp) {
        this.rtp = rtp;
        if (rtp) {
            resetPID();
        }
    }

    // Get run-to-position mode state
    public boolean getRtp() {
        return rtp;
    }

    // Set PID P coefficient
    public void setKP(double kP) {
        this.kP = kP;
    }

    // Set PID I coefficient and reset integral
    public void setKI(double kI) {
        this.kI = kI;
        resetIntegral();
    }

    // Set PID D coefficient
    public void setKD(double kD) {
        this.kD = kD;
    }

    // Set all PID coefficients
    public void setPidCoeffs(double kP, double kI, double kD) {
        setKP(kP);
        setKI(kI);
        setKD(kD);
    }

    // Get PID P coefficient
    public double getKP() {
        return kP;
    }

    // Get PID I coefficient
    public double getKI() {
        return kI;
    }

    // Get PID D coefficient
    public double getKD() {
        return kD;
    }

    // Set only P coefficient (alias)
    public void setK(double k) {
        setKP(k);
    }

    // Get only P coefficient (alias)
    public double getK() {
        return getKP();
    }

    // Set maximum allowed integral sum
    public void setMaxIntegralSum(double maxIntegralSum) {
        this.maxIntegralSum = maxIntegralSum;
    }

    // Get maximum allowed integral sum
    public double getMaxIntegralSum() {
        return maxIntegralSum;
    }

    // Get total rotation since initialization
    public double getTotalRotation() {
        return totalRotation;
    }

    // Get current target rotation
    public double getTargetRotation() {
        return targetRotation;
    }
    
    // Set compensation offset for direction changes
    public void setDirectionChangeCompensation(double compensation) {
        this.directionChangeCompensation = compensation;
    }
    
    // Get compensation offset
    public double getDirectionChangeCompensation() {
        return directionChangeCompensation;
    }

    // Increment target rotation by a value
    public void changeTargetRotation(double change) {
        // Detect direction change
        boolean directionChanged = (lastChangeDirection * change < 0) && (lastChangeDirection != 0);
        
        // Apply compensation if direction changed
        if (directionChanged) {
            targetRotation += change + (Math.signum(change) * directionChangeCompensation);
        } else {
            targetRotation += change;
        }
        
        lastChangeDirection = change;
    }

    // Set target rotation and reset PID
    public void setTargetRotation(double target) {
        targetRotation = target;
        resetPID();
    }

    // Get current angle from encoder (in degrees)
    public double getCurrentAngle() {
        if (servoEncoder == null)
            return 0;
        return (servoEncoder.getVoltage() / 3.3) * (direction.equals(Direction.REVERSE) ? 360 : -360);
    }

    // Check if servo is at target (default tolerance)
    public boolean isAtTarget() {
        return isAtTarget(5);
    }

    // Check if servo is at target (custom tolerance)
    public boolean isAtTarget(double tolerance) {
        return Math.abs(targetRotation - filteredTotalRotation) < tolerance;
    }

    // Normalize an angle to the range [-180, 180] so 360° == 0°
    private double wrapTo180(double angle) {
        double wrapped = angle % 360.0;
        if (wrapped > 180) {
            wrapped -= 360;
        } else if (wrapped < -180) {
            wrapped += 360;
        }
        return wrapped;
    }

    // Force reset total rotation and PID state
    public void forceResetTotalRotation() {
        totalRotation = 0;
        filteredTotalRotation = 0;
        previousAngle = getCurrentAngle();
        resetPID();
    }

    // Reset PID controller state
    public void resetPID() {
        resetIntegral();
        lastError = 0;
        pidTimer.reset();
    }

    // Reset integral sum
    public void resetIntegral() {
        integralSum = 0;
    }

    // Main update loop: updates rotation, computes PID, applies power
    public synchronized void update() {
        double currentAngle = getCurrentAngle();
        double angleDifference = currentAngle - previousAngle;

        // Handle wraparound at 0/360 degrees
        if (angleDifference > 180) {
            angleDifference -= 360;
            cliffs--;
        } else if (angleDifference < -180) {
            angleDifference += 360;
            cliffs++;
        }

        // Update total rotation with wraparound correction
        totalRotation = currentAngle - homeAngle + cliffs * 360;
        previousAngle = currentAngle;

        if (!rtp)
            return;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Ignore unreasonable dt values
        if (dt < 0.001 || dt > 1.0) {
            return;
        }

        // Low-pass filter to smooth totalRotation
        double alpha = 0.05;  // Smoothing factor (0.05-0.2 typical, lower = more smoothing)
        filteredTotalRotation += alpha * (totalRotation - filteredTotalRotation);

        // Use filtered position for PID calculation
        double error = targetRotation - totalRotation;
        // If target is effectively the same angle modulo 360, use the shorter path
        double wrappedError = wrapTo180(error);
        if (Math.abs(wrappedError) < Math.abs(error)) {
            error = wrappedError;
        }

        // Deadzone for output
        final double DEADZONE = 2;

        if (Math.abs(error) < DEADZONE) {
            error = 0;  // deadzone here
        }

        // PID integral calculation with clamping
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        // Integral wind-down in deadzone
        final double INTEGRAL_DEADZONE = 2.0;
        if (Math.abs(error) < INTEGRAL_DEADZONE) {
            integralSum *= 0.95;
        }

        // PID derivative calculation
        double derivative = (error - lastError) / dt;
        lastError = error;

        // PID output calculation
        double pTerm = kP * error;
        double iTerm = kI * integralSum;
        double dTerm = kD * derivative;

        double output = pTerm + iTerm + dTerm;



        if (Math.abs(error) > DEADZONE) {
            double power = Math.min(maxPower, Math.abs(output)) * Math.signum(output);
            setPower(power);
        } else {
            setPower(0);
        }

    }

    // Log current state for telemetry/debug
    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format(
                "Current Volts: %.3f\n" +
                        "Current Angle: %.2f\n" +
                        "Total Rotation: %.2f\n" +
                        "Target Rotation: %.2f\n" +
                        "Current Power: %.3f\n" +
                        "PID Values: P=%.3f I=%.3f D=%.3f\n" +
                        "PID Terms: Error=%.2f Integral=%.2f",
                servoEncoder.getVoltage(),
                getCurrentAngle(),
                totalRotation,
                targetRotation,
                power,
                kP, kI, kD,
                targetRotation - totalRotation,
                integralSum);
    }

    // TeleOp test class for manual tuning and testing
    @TeleOp(name = "Cont. Rotation Axon Test", group = "test")
    public static class CRAxonTest extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            // telemetry = new MultipleTelemetry(telemetry,
            // FtcDashboard.getInstance().getTelemetry());
            CRServo crservo = hardwareMap.crservo.get("indexer");
            CRServo crservo1 = hardwareMap.crservo.get("indexer1");
            AnalogInput encoder = hardwareMap.get(AnalogInput.class, "indexerEncoder");
            AnalogInput encoder1 = hardwareMap.get(AnalogInput.class, "indexerEncoder1");
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
            RTPAxon servo = new RTPAxon(crservo, encoder);
            RTPAxon servo1 = new RTPAxon(crservo1, encoder1);
            servo.setDirectionChangeCompensation(11);  // Start with 5 degrees
            servo1.setRtp(false);

            waitForStart();

            while (!isStopRequested()) {
                gamepads.copyStates();
                servo.update();
                double followerScale = 0.5;

                servo1.setRawPower(servo.getPower() * followerScale);

                // servo1.update();

                // Manual controls for target and PID tuning
                if (gamepads.isPressed(-1, "dpad_up")) {
                    servo.changeTargetRotation(60);
                    servo1.changeTargetRotation(60);
                }
                if (gamepads.isPressed(-1, "dpad_down")) {
                    servo.changeTargetRotation(-60);
                    servo1.changeTargetRotation(-60);
                }
                if (gamepads.isPressed(-1, "cross")) {
                    servo.setTargetRotation(0);
                    servo1.setTargetRotation(0);
                }

                if (gamepads.isPressed(-1, "triangle")) {
                    servo.setKP(servo.getKP() + 0.001);
                    servo1.setKP(servo1.getKP() + 0.001);
                }
                if (gamepads.isPressed(-1, "square")) {
                    servo.setKP(Math.max(0, servo.getKP() - 0.001));
                    servo1.setKP(Math.max(0, servo1.getKP() - 0.001));
                }

                if (gamepads.isPressed(-1, "dpad_left")) {
                    servo.setKD(servo.getKD() + 0.001);
                    servo1.setKD(servo1.getKD() + 0.001);
                }
                if (gamepads.isPressed(-1, "dpad_right")) {
                    servo.setKD(Math.max(0, servo.getKD() - 0.001));
                    servo1.setKD(Math.max(0, servo1.getKD() - 0.001));
                }

                if (gamepads.isPressed(-1, "right_bumper")) {
                    servo.setKI(servo.getKI() + 0.0001);
                    servo1.setKI(servo1.getKI() + 0.0001);
                }
                if (gamepads.isPressed(-1, "left_bumper")) {
                    servo.setKI(Math.max(0, servo.getKI() - 0.0001));
                    servo1.setKI(Math.max(0, servo1.getKI() - 0.0001));
                }

                // if (gamepads.isPressed(-1, "touchpad")) {
                // servo.setKP(0.015);
                // servo.setKI(0.0005);
                // servo.setKD(0.0025);
                // servo.resetPID();
                // }

                telemetry.addData("Starting angle", servo.STARTPOS);
                telemetry.addLine(servo.log());
                telemetry.addData("NTRY", servo.ntry);
                telemetry.addData("Starting angle1", servo1.STARTPOS);
                telemetry.addLine(servo1.log());
                telemetry.addData("NTRY1", servo1.ntry);
                telemetry.update();
            }
        }
    }
}
