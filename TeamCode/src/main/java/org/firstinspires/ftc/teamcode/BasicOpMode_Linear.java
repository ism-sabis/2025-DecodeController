package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;




@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class BasicOpMode_Linear extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private ColorSensor ColorSensor_ColorSensor;
    private DistanceSensor ColorSensor_DistanceSensor;

    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft   = null;
    private DcMotor rearRight  = null;

    @Override
    public void runOpMode() {

        int gain;
        boolean xButtonCurrentlyPressed;
        boolean xButtonPreviouslyPressed;
        NormalizedRGBA myNormalizedColors;
        int myColor;
        float hue;
        float saturation;
        float value;
        double Distance_cm_;


        ColorSensor_ColorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        ColorSensor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "ColorSensor");

        gain = 2;
        xButtonCurrentlyPressed = false;
        xButtonPreviouslyPressed = false;
        ColorSensor_ColorSensor.enableLed(true);
        Distance_cm_ = ColorSensor_DistanceSensor.getDistance(DistanceUnit.CM);
        waitForStart();



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
            Distance_cm_ = ColorSensor_DistanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.");
            telemetry.addLine(" ");
            telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value.");
            telemetry.addLine(" ");
            telemetry.addLine("Press the X button to turn the color sensor's LED on or off (if supported).");
            telemetry.addLine(" ");
            // Update the gain value if either of the A or B gamepad buttons is being held
            // A gain of less than 1 will make the values smaller, which is not helpful.
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain = (int) (gain + 0.005);
            } else if (gamepad1.b && gain > 1) {
                gain = (int) (gain - 0.005);
            }
            telemetry.addData("Gain", gain);
            // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
            ((NormalizedColorSensor) ColorSensor_ColorSensor).setGain(gain);
            xButtonCurrentlyPressed = gamepad1.x;
            // If the button state is different than what it was, then act
            // to turn the color sensor's light on or off (if supported).
            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                if (xButtonCurrentlyPressed) {
                    // If the button is (now) down, then toggle the light
                    ColorSensor_ColorSensor.enableLed(!((Light) ColorSensor_ColorSensor).isLightOn());
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;
            // Save the color sensor data as a normalized color value. It's recommended
            // to use Normalized Colors over color sensor colors is because Normalized
            // Colors consistently gives values between 0 and 1, while the direct
            // Color Sensor colors are dependent on the specific sensor you're using.
            myNormalizedColors = ((NormalizedColorSensor) ColorSensor_ColorSensor).getNormalizedColors();
            // Convert the normalized color values to an Android color value.
            myColor = myNormalizedColors.toColor();
            // Use the Android color value to calculate the Hue, Saturation and Value color variables.
            // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
            hue = JavaUtil.colorToHue(myColor);
            saturation = JavaUtil.colorToSaturation(myColor);
            value = JavaUtil.colorToValue(myColor);
            if (Distance_cm_ < 2) {
                // If this color sensor also has a distance sensor, display the measured distance.
                // Note that the reported distance is only useful at very close
                // range, and is impacted by ambient light and surface reflectivity.
                telemetry.addData("Loaded", 1);
            } else {
                // If this color sensor also has a distance sensor, display the measured distance.
                // Note that the reported distance is only useful at very close
                // range, and is impacted by ambient light and surface reflectivity.
                telemetry.addData("Loaded", 0);
            }
            if (hue >= 130 && hue <= 210) {
                // If this color sensor also has a distance sensor, display the measured distance.
                // Note that the reported distance is only useful at very close
                // range, and is impacted by ambient light and surface reflectivity.
                telemetry.addData("Green", 1);
            } else {
                // If this color sensor also has a distance sensor, display the measured distance.
                // Note that the reported distance is only useful at very close
                // range, and is impacted by ambient light and surface reflectivity.
                telemetry.addData("Green", 0);
            }
            if (hue >= 220 && hue <= 260) {
                // If this color sensor also has a distance sensor, display the measured distance.
                // Note that the reported distance is only useful at very close
                // range, and is impacted by ambient light and surface reflectivity.
                telemetry.addData("Purple", 1);
            } else {
                // If this color sensor also has a distance sensor, display the measured distance.
                // Note that the reported distance is only useful at very close
                // range, and is impacted by ambient light and surface reflectivity.
                telemetry.addData("Purple", 0);
            }
            // Use telemetry to display feedback on the driver station. We show the red,
            // green, and blue normalized values from the sensor (in the range of 0 to
            // 1), as well as the equivalent HSV (hue, saturation and value) values.
            telemetry.addLine("Red " + JavaUtil.formatNumber(myNormalizedColors.red, 3) + " | Green " + JavaUtil.formatNumber(myNormalizedColors.green, 3) + " | Blue " + JavaUtil.formatNumber(myNormalizedColors.blue, 3));
            telemetry.addLine("Hue " + JavaUtil.formatNumber(hue, 3) + " | Saturation " + JavaUtil.formatNumber(saturation, 3) + " | Value " + JavaUtil.formatNumber(value, 3));
            telemetry.addData("Alpha", Double.parseDouble(JavaUtil.formatNumber(myNormalizedColors.alpha, 3)));
            // If this color sensor also has a distance sensor, display the measured distance.
            // Note that the reported distance is only useful at very close
            // range, and is impacted by ambient light and surface reflectivity.
            telemetry.addData("Distance (cm)", Double.parseDouble(JavaUtil.formatNumber(ColorSensor_DistanceSensor.getDistance(DistanceUnit.CM), 3)));
            telemetry.update();
        }
    }
}