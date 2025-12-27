package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class RobotHardware {

    // Define Motor and Servo objects (Make them private so they can't be accessed
    // externally)
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
    public CRServo indexer = null;

    public CRServo indexer1 = null;
    public Limelight3A limelight = null;

    public NormalizedColorSensor colorSensor = null;
    public NormalizedColorSensor colorSensor1 = null;

    // This declares the IMU needed to get the current direction the robot is facing
    public IMU imu;

    public static final double limelightHeight = 20;

    // Define Drive constants. Make them public so they CAN be used by the calling
    // OpMode
    public static final double frontLeftPower = 0.5;
    public static final double backLeftPower = 0.02;
    public static final double frontRightPower = 0.45;
    public static final double backRightPower = -0.45;

    /*
     * // Constants
     * // public static final double SPEED_LIMIT = 0.5;
     * public static final double LAUNCHER_SPEED_DEFAULT = 0.8;
     * public static final double KICKER_POSITION_START = 0.0;
     * // public static final double FEEDER_ROTATION_SPEED = 1;
     * public static final double LIFT_SPEED_DEFAULT = 1;
     *
     *
     * // Variables
     * //double maxSpeed;
     * int intakeRuntime;
     * boolean kickerTf;
     * float yaw;
     * ElapsedTime runtime;
     * int liftSpeed;
     * DcMotor feederRotation; // replace with correct type if not a motor
     * double launcherSpeed;
     * double kickerPosition;
     * float axial;
     * float lateral;
     */

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware() {
        // myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and
     * initialized.
     */
    public void init(HardwareMap hardwareMap) {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        feedingRotation = hardwareMap.get(DcMotor.class, "feedingRotation");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        imu = hardwareMap.get(IMU.class, "imu");

        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");

        // Define and initialize ALL installed servos.
        kicker = hardwareMap.get(Servo.class, "Kicker");

        turretSpinner = hardwareMap.get(CRServo.class, "turretSpinner");
        turretSpinner.setPower(0); // stop initially

        indexer = hardwareMap.get(CRServo.class, "indexer");
        indexer.setPower(0); // stop initially

        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer1.setPower(0); // stop initially

        // To drive forward, most robots need the motor on one side to be reversed,
        // because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these
        // two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear
        // Reduction or 90 Deg drives may require direction flips
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        turretSpinner.setDirection(CRServo.Direction.FORWARD);
        feedingRotation.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater
        // accuracy
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feedingRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        feedingRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // ZeroPowerBehavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Starts polling for data. If you neglect to call start(), getLatestResult()
        // will return null.
        limelight.pipelineSwitch(0);
        limelight.start();

        /*
         * leftHand.setPosition(MID_SERVO);
         * rightHand.setPosition(MID_SERVO);
         */

        // myOpMode.telemetry.addData(">", "Hardware Initialized");
        // myOpMode.telemetry.update();

        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn  Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
        double left = Drive + Turn;
        double right = Drive - Turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(left, right);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive
     * motors.
     *
     * @param leftWheel  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        rearLeft.setPower(leftWheel);
        frontLeft.setPower(leftWheel);
        rearRight.setPower(rightWheel);
        frontRight.setPower(rightWheel);
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    /*
     * public void setArmPower(double power) {
     * armMotor.setPower(power);
     * }
     */

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the
     * passed offset.
     *
     * @param offset
     */
    /*
     * public void setHandPositions(double offset) {
     * offset = Range.clip(offset, -0.5, 0.5);
     * leftHand.setPosition(MID_SERVO + offset);
     * rightHand.setPosition(MID_SERVO - offset);
     * }
     */
}
