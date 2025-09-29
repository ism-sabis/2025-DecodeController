package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class BasicOpMode_Linear extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft   = null;
    private DcMotor rearRight  = null;

    @Override
    public void runOpMode() {

        // TODO: replace these strings with the EXACT names in your RC configuration
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft   = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight  = hardwareMap.get(DcMotor.class, "rearRight");

        // Set motor directions so pushing left stick forward drives forward
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // POV drive: left stick Y = forward/back, right stick X = turn
            double drive = -gamepad1.left_stick_y;  // push forward -> positive
            double turn  =  gamepad1.right_stick_x;

            // Combine for left/right sides
            double left  = drive + turn;
            double right = drive - turn;

            // Normalize so no value exceeds 1.0
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left  /= max;
                right /= max;
            }

            double frontLeftPower  = left;
            double rearLeftPower   = left;
            double frontRightPower = right;
            double rearRightPower  = right;

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(rearLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left  (FL/RL)",  "%.2f / %.2f", frontLeftPower,  rearLeftPower);
            telemetry.addData("Right (FR/RR)", "%.2f / %.2f", frontRightPower, rearRightPower);
            telemetry.update();
        }
    }
}