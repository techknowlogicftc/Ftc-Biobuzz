package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * LaunchTestTeleOp - A minimal TeleOp program for testing launch speeds
 *
 * This program provides ONLY launcher functionality for testing and tuning:
 * - Toggle launcher motors on/off with gamepad1.y
 * - Uses the launch constants defined at the top of this class
 *
 * TO ADJUST SPEEDS:
 * Edit the constants at the top of this class:
 * - FAR_LAUNCH_SPEED: Speed used when the launcher is toggled on
 * - NEAR_LAUNCH_SPEED: Full speed for normal launch (teleop)
 *
 * CONTROLS:
 * - gamepad1.y: Toggle launcher motors on/off
 * - gamepad1.x / a / b: Run launcher at FAR_LAUNCH_SPEED / FAR_LAUNCH_SPEED2 / FAR_LAUNCH_SPEED3
 * - gamepad1.right_trigger: Hold to run launcher proportionally up to FAR_LAUNCH_SPEED4; release to stop
 */

@TeleOp(name = "Launch Test", group = "Test")
public class LaunchTestTeleOp extends OpMode {
    // Launch constants (copied from Decode_Pedro RobotConstants)
    public static final int NEAR_LAUNCH_SPEED = 2080;         // Full speed for normal launch (teleop)
    public static final int FAR_LAUNCH_SPEED = 300;          // High speed for small triangle launch (teleop)
    public static final int FAR_LAUNCH_SPEED2 = 2000;

    public static final int FAR_LAUNCH_SPEED3 = 2500;

    public static final int FAR_LAUNCH_SPEED4 = 3000;          // High speed for small triangle launch (teleop)
// High speed for small triangle launch (teleop)
// High speed for small triangle launch (teleop)

    public static final int LAUNCHER_STOP_SPEED = 0;          // Stop speed
    public static final double TRIGGER_DEADBAND = 0.05;       // Ignore trigger values below this

    // Hardware components
    protected DcMotorEx launch1;      // Left launcher motor
    protected DcMotorEx launch2;      // Right launcher motor

    protected boolean launcherOn = false;  // Tracks whether launcher motors are on
    protected boolean triggerActive = false;  // Tracks whether right trigger is controlling speed

    @Override
    public void init() {
        // Initialize hardware components
        launch1 = hardwareMap.get(DcMotorEx.class, "launchleft");
        launch2 = hardwareMap.get(DcMotorEx.class, "launchright");

        // Configure launcher motors for encoder-based velocity control
        launch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch1.getMotorType().setTicksPerRev(28);
        launch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop launcher motors
        launch1.setVelocity(LAUNCHER_STOP_SPEED);
        launch2.setVelocity(LAUNCHER_STOP_SPEED);

        telemetry.addData("Status", "Launch Test TeleOp Ready");
        telemetry.addData("Controls", "Y=toggle launcher, X/A/B=set speed, RT=variable speed");
        telemetry.addData("Launcher", "OFF");
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset state when opmode starts
        launch1.setVelocity(LAUNCHER_STOP_SPEED);
        launch2.setVelocity(LAUNCHER_STOP_SPEED);
    }

    @Override
    public void loop() {
        // Toggle launcher motors with Y button
        if (gamepad1.yWasPressed()) {
            launcherOn = !launcherOn;
            if (launcherOn) {
                launch1.setVelocity(FAR_LAUNCH_SPEED);
                launch2.setVelocity(FAR_LAUNCH_SPEED);
                telemetry.addData("Status", "Launcher motors ON");
            } else {
                launch1.setVelocity(LAUNCHER_STOP_SPEED);
                launch2.setVelocity(LAUNCHER_STOP_SPEED);
                telemetry.addData("Status", "Launcher motors OFF");
            }
        }

        // Select launcher speed with X, A or B
        if (gamepad1.xWasPressed()) {
            setLaunchSpeed(FAR_LAUNCH_SPEED);
        } else if (gamepad1.aWasPressed()) {
            setLaunchSpeed(FAR_LAUNCH_SPEED2);
        } else if (gamepad1.bWasPressed()) {
            setLaunchSpeed(FAR_LAUNCH_SPEED3);
        }

        // Right trigger sets launcher speed proportionally (0 to FAR_LAUNCH_SPEED4)
        if (gamepad1.right_trigger > TRIGGER_DEADBAND) {
            triggerActive = true;
            setLaunchSpeed((int) (gamepad1.right_trigger * FAR_LAUNCH_SPEED4));
        } else if (triggerActive) {
            // Trigger released - stop launcher
            triggerActive = false;
            launcherOn = false;
            launch1.setVelocity(LAUNCHER_STOP_SPEED);
            launch2.setVelocity(LAUNCHER_STOP_SPEED);
            telemetry.addData("Status", "Trigger released - Launcher motors OFF");
        }

        // Update telemetry
        updateTelemetry();
    }

    public  double getRpm(){
        double rpm = 0.0;
        if(launcherOn) {
            double ticksPerRev = launch1.getMotorType().getTicksPerRev();

            // Get velocity in ticks per second
            double ticksPerSec = launch1.getVelocity();

            // Convert ticks per second to Revolutions Per Minute (RPM)
            rpm = (ticksPerSec / ticksPerRev) * 60.0;
        }
        return rpm;
    }

    /**
     * Run both launcher motors at the given velocity
     */
    protected void setLaunchSpeed(int speed) {
        launcherOn = true;
        launch1.setVelocity(speed);
        launch2.setVelocity(speed);
        telemetry.addData("Status", String.format("Launcher set to %d ticks/sec", speed));
    }

    /**
     * Update telemetry display
     */
    protected void updateTelemetry() {
        telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
        telemetry.addData("Launcher Speed", launch1.getVelocity() > 0 ?
            String.format("%d ticks/sec", (int)launch1.getVelocity()) : "STOPPED");
        telemetry.addData("Current RPM", getRpm());
        telemetry.addData("ticksPerRev", launch1.getMotorType().getTicksPerRev());
        //telemetry.addData('ticksPerRev', launch1.getMotorType().getTicksPerRev());
        telemetry.addData("---", "---");

        telemetry.addData("Speeds:", "");
        //telemetry.addData("  Full Speed", String.format("%d ticks/sec", NEAR_LAUNCH_SPEED));
        telemetry.addData("---", "---");
        telemetry.addData("Controls", "Y=toggle launcher, X/A/B=set speed, RT=variable speed");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Cleanup when opmode stops
        launch1.setVelocity(LAUNCHER_STOP_SPEED);
        launch2.setVelocity(LAUNCHER_STOP_SPEED);
    }
}
