package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo; // Import CRServo
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A class to manage the robot's intake mechanism.
 * This class encapsulates all hardware and logic for taking balls into the robot,
 * controlling two DC Motors and one CRServo.
 */
@TeleOp(name = "Shooter TeleOp", group = "Sensor")
public class Shooter extends OpMode{

        // --- Hardware Members ---
        private CRServo intakeGate;     // Changed from Servo to CRServo for goBILDA Dual Mode
        private DcMotor conveyorBelt;   //  DC motor, to shoot the ball likely for a conveyor system.

        // --- Constants ---
        // Define the power for the DC motors when running. (0.0 to 1.0)
        private static final double INTAKE_POWER = 1.0;

        // Define the power for the CRServo.
        private static final double GATE_INTAKE_POWER = 1.0;  // Power for continuous rotation (e.g., clockwise)
        private static final double GATE_OUTTAKE_POWER = -1.0; // Power for opposite direction
        private static final double GATE_STOP_POWER = 0.0;    // Power to make the servo stop

        @Override
        public void init() {
            // Initialize the intake system by passing it the hardwareMap
            initiateHardware(hardwareMap);

            telemetry.addLine("Robot Initialized. Intake will run automatically on START.");
            telemetry.update();
        }

        @Override
        public void loop() {
            // --- Intake Control ---
            // The intake will now run continuously by default after START is pressed.
            // All gamepad logic has been removed from this method.
            runIntake();

            // Update the telemetry to show the current, constant status.
            telemetry.addData("Intake Status", "ON (Running Automatically)");
            telemetry.update();
        }

        /**
         * Initializes all the hardware components for the intake system.
         * @param hardwareMap The hardware map from the OpMode, used to get the devices.
         */
        public void initiateHardware(HardwareMap hardwareMap) {
            // Get the motors and servo from the robot's configuration.
            // Make sure the names here match your robot's config file exactly.
            // Get the servo as a CRServo
            intakeGate = hardwareMap.get(CRServo.class, "intake_gate");
        conveyorBelt = hardwareMap.get(DcMotor.class, "conveyor_belt");

            // --- Set Motor Directions ---
            // "Clockwise" is ambiguous for DC motors as it depends on how they are mounted.
            // We define "intake" as the positive power direction. You may need to change
            // DcMotorSimple.Direction.FORWARD to .REVERSE for a motor if it spins the wrong way
            // during your first test. This is a common and expected tuning step.

            // Let's assume FORWARD power on both motors makes them spin to pull balls in.
            conveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);
            // Also set the direction for the CRServo. You may need to reverse this.
            intakeGate.setDirection(DcMotorSimple.Direction.FORWARD);


            // It's good practice to ensure all motors are stopped on initialization.
            stop();
        }

        /**
         * Turns on the intake system to pull balls into the robot.
         * - Spins the roller motor.
         * - Spins the conveyor belt motor.
         * - Spins the gate servo to assist with intake.
         */
        public void runIntake() {
            conveyorBelt.setPower(INTAKE_POWER);
            intakeGate.setPower(GATE_INTAKE_POWER);
        }

        /**
         * Turns off the entire intake system.
         * - Stops both DC motors.
         * - Stops the gate servo.
         */
        public void stop() {
            conveyorBelt.setPower(0);
            intakeGate.setPower(GATE_STOP_POWER);
        }


    }
