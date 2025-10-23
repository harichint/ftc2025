package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import android.graphics.Color;

@TeleOp(name = "Ball Color Recognition", group = "Sensor")
public class BallColorRecognition extends LinearOpMode {

    // Define a variable for the color sensor
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Get the color sensor from the hardware map
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // --- INCREASE SENSOR SENSITIVITY (GAIN) ---
        colorSensor.setGain(16.0f); // Set gain to 16. Tune this value!

        // Wait for the start button to be pressed
        waitForStart();

        // Loop while the OpMode is active
        while (opModeIsActive()) {
            // Get normalized color values
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to HSV for easier color detection
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            // Extract Hue
            float hue = hsvValues[0];

            String detectedColor = "Unknown";

            // Define Hue ranges for purple and green
            // These values may need tuning based on your specific sensor and lighting conditions
            if (hue >= 90 && hue <= 167) { // Approximate range for green
                detectedColor = "Green";
            } else if (hue >= 200 && hue <= 280) { // Approximate range for purple
                detectedColor = "Purple";
            }

            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Hue", "%.2f", hue);
            telemetry.update();
        }
    }
}
