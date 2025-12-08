package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "DriverNew: Joystick Drive", group = "Drivetrain") // Updated name for clarity
public class DriverNew extends OpMode {

    // --- Hardware Members ---
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    @Override
    public void init() {
        // Initialize all drivetrain motors using the helper method
        initDrivetrain(hardwareMap);
        telemetry.addLine("Ready for Joystick Control!");
        telemetry.addLine("> Left Stick: Forward/Backward/Strafe");
        telemetry.addLine("> Right Stick: Rotate");
        telemetry.update();
    }

    @Override
    public void loop() {
        // =============================== THE FIX ===============================
        // This is the standard Mecanum drive logic that uses joysticks.

        // The `-` sign on left_stick_y is because the joystick's Y-axis is inverted
        // (pushing forward gives a negative value).
        double forward = -gamepad1.left_stick_y;  // Controls forward and backward movement
        double strafe  =  gamepad1.left_stick_x;  // Controls left and right strafing
        double rotate  =  gamepad1.right_stick_x;  // Controls robot rotation

        // --- Mecanum Drive Formulas ---
        // These formulas combine the three inputs to calculate the power for each wheel.
        double leftFrontPower  = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower   = forward - strafe + rotate;
        double rightBackPower  = forward + strafe - rotate;

        // --- Normalization ---
        // This is a critical step. It finds the largest motor power (absolute value)
        // or 1, and divides all motor powers by it. This ensures that no motor power
        // ever goes above 1.0 while maintaining the correct drive proportions.
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        leftFront.setPower(leftFrontPower / denominator);
        rightFront.setPower(rightFrontPower / denominator);
        leftBack.setPower(leftBackPower / denominator);
        rightBack.setPower(rightBackPower / denominator);

        // =======================================================================

        // Telemetry to show what the robot is doing
        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Rotate", "%.2f", rotate);
        telemetry.addLine("---");
        telemetry.addData("Front L/R", "%.2f, %.2f", leftFront.getPower(), rightFront.getPower());
        telemetry.addData("Back  L/R", "%.2f, %.2f", leftBack.getPower(), rightBack.getPower());
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

        /*
         =============================== THE FIX ===============================
         * For a standard mecanum drive, all the motors on one side of the robot
         * need to be reversed. The other side should be set to forward.
         *
         * Let's reverse the entire left side.
         =======================================================================
         */
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the right side to forward (this is the default, but it's good to be explicit)
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}
