package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * Controls:
 * - Left Stick Y: Forward/Backward
 * - Right Stick X: Drift left/right (strafe)
 * - A (hold): Run only the rear left motor
 */
@TeleOp
public class MecanumDriveTeleOp extends OpMode {

    private MecanumDrive drive;

    private static final double DEADZONE = 0.1;
    private static final double TEST_MOTOR_POWER = 0.5;

    @Override
    public void init() {
        drive = new MecanumDrive();
        drive.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Stick Y is negative when pushed forward, so negate it to make forward positive
        double forward = -applyDeadzone(gamepad1.left_stick_y, DEADZONE);
        double strafe = applyDeadzone(gamepad1.right_stick_x, DEADZONE);

        if (gamepad1.a) {
            // Hold A to spin only the rear left motor
            drive.runRearLeft(TEST_MOTOR_POWER);
        } else {
            drive.drive(forward, strafe);
        }

        telemetry.addData("Rear Left Test (A)", gamepad1.a ? "RUNNING" : "off");
        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.update();
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        return value;
    }

    @Override
    public void stop() {
        drive.stopMotors();
    }
}
