package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * Controls:
 * - Left Stick Y: Forward/Backward
 * - Left Stick X: Drift left/right (strafe)
 * - Right Stick X: Turn left/right
 * - Right Trigger (hold): Slow mode (1/4 speed)
 * - A (hold): Run only the rear left motor
 */
@TeleOp
public class MecanumDriveTeleOp extends OpMode {

    private MecanumDrive drive;

    private static final double DEADZONE = 0.1;
    private static final double TEST_MOTOR_POWER = 0.5;
    private static final double SLOW_MODE_MULTIPLIER = 0.25;

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
        double strafe = applyDeadzone(gamepad1.left_stick_x, DEADZONE);
        double turn = applyDeadzone(gamepad1.right_stick_x, DEADZONE);

        boolean slowMode = gamepad1.right_trigger > DEADZONE;
        double speedMultiplier = slowMode ? SLOW_MODE_MULTIPLIER : 1.0;
        forward *= speedMultiplier;
        strafe *= speedMultiplier;
        turn *= speedMultiplier;

        if (gamepad1.a) {
            // Hold A to spin only the rear left motor
            drive.runRearLeft(TEST_MOTOR_POWER);
        } else {
            drive.drive(forward, strafe, turn);
        }

        telemetry.addData("Rear Left Test (A)", gamepad1.a ? "RUNNING" : "off");
        telemetry.addData("Slow Mode (RT)", slowMode ? "ON (1/4 speed)" : "off");
        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
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
