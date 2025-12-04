package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "DriverNew: Combined Drivetrain", group = "Drivetrain")
public class DriverNew extends OpMode {

    // --- Hardware Members ---
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // --- Constants ---
    private static final double DRIVE_POWER = 0.5;  // Base power for forward/reverse
    private static final double STRAFE_POWER = 0.6; // Base power for strafing

    @Override
    public void init() {
        // Initialize all drivetrain motors using the helper method
        initDrivetrain(hardwareMap);
        telemetry.addLine("Ready to Drive!");
        telemetry.addLine("> Y: Forward");
        telemetry.addLine("> A: Reverse");
        telemetry.addLine("> X: Strafe Left");
        telemetry.addLine("> B: Strafe Right");
        telemetry.update();
    }

    @Override
    public void loop() {
        // This if-else if structure ensures that only one movement command is processed at a time.
        if (gamepad1.y) {
            // Drive Forward
            leftFront.setPower(DRIVE_POWER);
            rightFront.setPower(DRIVE_POWER);
            leftBack.setPower(DRIVE_POWER);
            rightBack.setPower(DRIVE_POWER);
            telemetry.addData("Mode", "Driving Forward");

        } else if (gamepad1.a) {
            // Drive Reverse
            leftFront.setPower(-DRIVE_POWER);
            rightFront.setPower(-DRIVE_POWER);
            leftBack.setPower(-DRIVE_POWER);
            rightBack.setPower(-DRIVE_POWER);
            telemetry.addData("Mode", "Driving Reverse");

        } else if (gamepad1.x) {
            // Strafe Left
            leftFront.setPower(-STRAFE_POWER);
            rightFront.setPower(STRAFE_POWER);
            leftBack.setPower(STRAFE_POWER);
            rightBack.setPower(-STRAFE_POWER);
            telemetry.addData("Mode", "Strafing Left");

        } else if (gamepad1.b) {
            // Strafe Right
            leftFront.setPower(STRAFE_POWER);
            rightFront.setPower(-STRAFE_POWER);
            leftBack.setPower(-STRAFE_POWER);
            rightBack.setPower(STRAFE_POWER);
            telemetry.addData("Mode", "Strafing Right");

        } else {
            // If no drive buttons are pressed, stop all motors.
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            telemetry.addData("Mode", "Stopped");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure motors are stopped when the OpMode ends
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    /**
     * Helper method to initialize the 4 drivetrain motors.
     * @param hwMap The hardware map from the OpMode.
     */
    private void initDrivetrain(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotor.class, "left_front_drive");
        rightFront = hwMap.get(DcMotor.class, "right_front_drive");
        leftBack = hwMap.get(DcMotor.class, "left_back_drive");
        rightBack = hwMap.get(DcMotor.class, "right_back_drive");

        // Set motor directions. For a standard Mecanum or Tank drive, one side needs to be reversed.
        // TUNE THIS based on your robot's behavior.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
