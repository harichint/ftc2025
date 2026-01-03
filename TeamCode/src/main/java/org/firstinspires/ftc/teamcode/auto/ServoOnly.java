package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * This Autonomous OpMode automatically starts the CRServo at full power.
 */
@Autonomous(name = "Servo Only (Auto)", group = "Autonomous")
public class ServoOnly extends LinearOpMode {

    // --- HARDWARE ---
    private CRServo intakeGate;

    // --- CONSTANTS ---
    // Using 1.0 for maximum power.
    private static final double MAX_POWER = 1.0;

    @Override
    public void runOpMode() {
        // 1. Initialize the servo using the name from your config
        intakeGate = hardwareMap.get(CRServo.class, "intake_gate");

        telemetry.addLine("Servo Initialized.");
        telemetry.addData("Status", "Ready to Start");
        telemetry.update();

        // 2. Wait for the driver to press the START button
        waitForStart();

        if (opModeIsActive()) {
            // 3. Set the servo to high power
            intakeGate.setPower(MAX_POWER);

            telemetry.addData("Status", "Running at High Power");
            telemetry.update();

            // 4. Keep running while the OpMode is active
            // This prevents the code from finishing and stopping the motor immediately
            while (opModeIsActive()) {
                idle(); // Give other system threads a chance to run
            }
        }

        // 5. Ensure the servo stops when the OpMode is finished
        intakeGate.setPower(0);
    }
}
