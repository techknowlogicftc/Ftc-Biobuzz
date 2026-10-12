package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.IMUUtil;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightUtil;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * Autonomous: drive straight forward for a set time, turn right 90 degrees with the IMU,
 * auto-align to an AprilTag with the Limelight, then turn parallel to the tag.
 *
 * Sign conventions follow MecanumDrive.drive(forward, strafe, turn):
 * - Positive forward = drive forward
 * - Positive turn = spin right
 * IMU heading is the opposite: positive = left, negative = right.
 */
@Autonomous
public class DriveStraightTimedAuto extends LinearOpMode {

    private MecanumDrive drive;
    private IMUUtil imu;
    private LimelightUtil limelight;

    // Starting values for this robot - not yet tuned on the mecanum drive
    private static final double DRIVE_SECONDS = 2.2;
    private static final double DRIVE_POWER = 0.3;
    private static final double DRIVE_HEADING_KP = 0.01;  // turn power per degree of drift; flip sign if it veers more

    // Turn settings
    private static final double TURN_DEGREES = -90.0;      // IMU heading: positive = left, negative = right
    private static final double TURN_KP = 0.0075;          // power per degree of heading error
    private static final double TURN_KD = 0.0015;          // damping per deg/sec of spin; raise if it still overshoots
    private static final double TURN_MIN_POWER = 0.12;     // enough to overcome friction; raise if it stops short
    private static final double TURN_MAX_POWER = 0.4;      // lower if it overshoots
    private static final double HEADING_TOLERANCE = 1.5;   // degrees
    private static final double SETTLE_SECONDS = 0.25;     // must stay in tolerance this long to finish
    private static final double TURN_TIMEOUT_SECONDS = 3.0;  // time to correct back if it overshoots
    private static final long PAUSE_BETWEEN_STEPS_MS = 300;  // full stop between driving and turning

    // Auto-align settings
    private static final double ALIGN_TOLERANCE = 4.0;      // degrees of tx
    private static final double ALIGN_KP = 0.005;           // power per degree of tx
    private static final double ALIGN_KD = 0.00075;        // damping per deg/sec of tx change
    private static final double ALIGN_MIN_POWER = 0.12;
    private static final double ALIGN_MAX_POWER = 0.3;
    private static final double CAMERA_TX_OFFSET = 2.35;    // tx reading when robot is truly centered
    private static final double ALIGN_TIMEOUT_SECONDS = 3.0;
    private final ElapsedTime alignTimer = new ElapsedTime();
    private double lastTxError = Double.NaN;  // NaN = no previous reading, skip damping

    // Parallel-to-tag settings - calibrate on the robot using the "Tag Yaw (live)" telemetry
    private static final double TAG_YAW_WHEN_SQUARE = 0.0;  // tag yaw reading when robot faces the tag squarely (0 or 180)
    private static final double TAG_YAW_SIGN = -1.0;        // -1 if tag yaw goes UP when robot turns left, +1 if it goes DOWN
    private static final double PARALLEL_TURN_DEGREES = 60.0;  // +90 = parallel turned left, -90 = turned right
    private static final int TAG_YAW_SAMPLES = 10;          // average this many readings (pose yaw is noisy)


    @Override
    public void runOpMode() {
        drive = new MecanumDrive();
        drive.init(hardwareMap);

        imu = new IMUUtil();
        imu.init(hardwareMap);

        limelight = new LimelightUtil(hardwareMap);
        limelight.setTargetTagId(-1);  // align to any visible tag

        telemetry.addData("Status", "Initialized - press START");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        imu.resetHeading();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
            double headingError = imu.getHeadingDifference(0, imu.getHeading());  // negative = drifted left
            // Steer back toward heading 0 while driving
            double correction = Range.clip(-headingError * DRIVE_HEADING_KP, -0.2, 0.2);

            drive.drive(DRIVE_POWER, 0, correction);

            telemetry.addData("Step", "Driving");
            telemetry.addData("Time", String.format("%.1f / %.1f s", timer.seconds(), DRIVE_SECONDS));
            telemetry.addData("Heading Error", String.format("%.1f°", headingError));
            telemetry.update();
        }

        drive.stopMotors();
        /*
            sleep(PAUSE_BETWEEN_STEPS_MS);
            String turnResult = turnToHeading(TURN_DEGREES);
            drive.stopMotors();
        */

        while (opModeIsActive()) {
            telemetry.addData("Status", "Done");
            telemetry.addData("Final Heading", String.format("%.1f°", imu.getHeading()));
            // Live reading for calibrating TAG_YAW_WHEN_SQUARE and TAG_YAW_SIGN:
            // move the robot by hand while the tag is in view and watch this number
            limelight.update();
            telemetry.addData("Tag Yaw (live)", limelight.hasTarget()
                    ? String.format("%.1f°", limelight.getTagYaw()) : "no tag");
            telemetry.update();
            sleep(50);
        }
        limelight.stop();
    }

    /**
     * Turn so the robot is parallel to the AprilTag's surface.
     *
     * tx can't do this (it is ~0 after auto-align whatever angle the robot is at), so this
     * reads the tag's 3D yaw to find the heading where the robot would face the tag squarely,
     * then uses the IMU to turn from that. The IMU does the turn because the tag
     * leaves the camera's view partway through.
     *
     * @param sideDegrees +90 to end up parallel turned left, -90 turned right
     * @return a short description of how the step ended, for telemetry
     */
    private String turnParallelToTag(double sideDegrees) {
        // Average several readings while the tag is still in view
        double yawSum = 0;
        int samples = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && samples < TAG_YAW_SAMPLES && timer.seconds() < 1.0) {
            limelight.update();
            if (limelight.hasTarget()) {
                // Offset from "square", wrapped to -180..180 so readings near 180 average correctly
                yawSum += imu.getHeadingDifference(limelight.getTagYaw(), TAG_YAW_WHEN_SQUARE);
                samples++;
            }
            sleep(20);
        }
        if (samples == 0) {
            return "NO TAG - parallel turn skipped";
        }
        double yawFromSquare = yawSum / samples;

        // Heading where the robot would face the tag straight on, then turn from it
        double squareHeading = imu.getHeading() + TAG_YAW_SIGN * yawFromSquare;
        double targetHeading = imu.getHeadingDifference(squareHeading + sideDegrees, 0);  // wrap to -180..180

        String result = turnToHeading(targetHeading);
        return String.format("tag yaw %.1f° -> target %.1f°: %s", yawFromSquare, targetHeading, result);
    }


    /**
     * Turn in place until the AprilTag is centered in the Limelight view.
     *
     * @return a short description of how the alignment ended, for telemetry
     */
    private String autoAlign() {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        lastTxError = Double.NaN;
        alignTimer.reset();

        while (opModeIsActive()) {
            if (timer.seconds() >= ALIGN_TIMEOUT_SECONDS) {
                drive.stopMotors();
                return limelight.hasTarget()
                        ? String.format("TIMED OUT at tx %.1f°", limelight.getTx() - CAMERA_TX_OFFSET)
                        : "NO TAG SEEN - check the tag is in view after the turn";
            }

            // Must be called every loop before reading Limelight data
            limelight.update();

            double turn = 0.0;
            if (!limelight.hasTarget()) {
                // Don't guess which way to turn - wait for the tag
                lastTxError = Double.NaN;
                settleTimer.reset();
            } else {
                double txError = limelight.getTx() - CAMERA_TX_OFFSET;
                turn = calculateAlignCorrection(txError);

                if (Math.abs(txError) <= ALIGN_TOLERANCE) {
                    if (settleTimer.seconds() >= SETTLE_SECONDS) {
                        drive.stopMotors();
                        return String.format("Aligned to tag %d (tx %.1f°)", limelight.getTrackedTagId(), txError);
                    }
                } else {
                    settleTimer.reset();
                }
            }

            drive.drive(0, 0, turn);

            telemetry.addData("Step", "Auto-Align");
            telemetry.addData("Tag", limelight.hasTarget() ? limelight.getTrackedTagId() : "none");
            telemetry.addData("tx", String.format("%.1f°", limelight.getTx() - CAMERA_TX_OFFSET));
            telemetry.addData("Turn Power", String.format("%.2f", turn));
            telemetry.update();
        }
        drive.stopMotors();
        return "Stopped";
    }

    /**
     * Proportional turn with damping, a minimum power so it doesn't stall,
     * and a tolerance band so it stops cleanly.
     * Positive tx = tag is to the right = positive (right) turn.
     */
    private double calculateAlignCorrection(double txError) {
        // Damping: push against how fast tx is changing so the robot slows before center
        double dt = alignTimer.seconds();
        alignTimer.reset();
        double txRate = (Double.isNaN(lastTxError) || dt <= 0) ? 0.0 : (txError - lastTxError) / dt;
        lastTxError = txError;

        if (Math.abs(txError) <= ALIGN_TOLERANCE) {
            return 0.0;
        }
        double proportional = txError * ALIGN_KP;
        double turn = proportional + txRate * ALIGN_KD;
        // Only boost to min power when still heading toward the tag, not while damping is braking
        if (Math.abs(turn) < ALIGN_MIN_POWER && Math.signum(turn) == Math.signum(proportional)) {
            turn = Math.copySign(ALIGN_MIN_POWER, turn);
        }
        return Range.clip(turn, -ALIGN_MAX_POWER, ALIGN_MAX_POWER);
    }


    /**
     * Turn in place to the given heading (relative to where the robot started).
     * Positive = left, negative = right.
     *
     * @return a short description of how the turn ended, for telemetry
     */
    private String turnToHeading(double targetHeading) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        double startHeading = imu.getHeading();
        double startError = Math.abs(imu.getHeadingDifference(targetHeading, startHeading));

        // Keep correcting until the heading is on target (or we time out)
        while (opModeIsActive()) {
            if (timer.seconds() >= TURN_TIMEOUT_SECONDS) {
                drive.stopMotors();
                return String.format("TIMED OUT at %.1f°", imu.getHeading());
            }

            double heading = imu.getHeading();
            // Target stays fixed; only the error changes as the robot turns
            double error = imu.getHeadingDifference(targetHeading, heading);  // negative = need to turn right

            // Safety checks: stop instead of spinning in circles
            if (Math.abs(error) > startError + 20) {
                drive.stopMotors();
                return String.format("WRONG WAY (heading %.1f°) - flip the sign of TURN_KP", heading);
            }
            if (timer.seconds() > 0.75 && Math.abs(imu.getHeadingDifference(heading, startHeading)) < 2) {
                drive.stopMotors();
                return "IMU NOT SEEING TURN - check hub orientation in IMUUtil.init, or raise TURN_MIN_POWER";
            }

            double turn = 0.0;
            if (Math.abs(error) <= HEADING_TOLERANCE) {
                if (settleTimer.seconds() >= SETTLE_SECONDS) {
                    drive.stopMotors();
                    return String.format("Reached %.1f°", heading);
                }
            } else {
                settleTimer.reset();
                double proportional = -error * TURN_KP;  // error -90 (need right) -> positive turn -> spins right
                // Damping: push against the current spin so the robot slows down before the target
                double spinRate = imu.getIMU().getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;  // + = turning left
                turn = proportional + spinRate * TURN_KD;
                // Only boost to min power when still heading toward the target, not while damping is braking
                if (Math.abs(turn) < TURN_MIN_POWER && Math.signum(turn) == Math.signum(proportional)) {
                    turn = Math.copySign(TURN_MIN_POWER, turn);
                }
                turn = Range.clip(turn, -TURN_MAX_POWER, TURN_MAX_POWER);
            }

            drive.drive(0, 0, turn);

            telemetry.addData("Step", "Turning");
            telemetry.addData("Heading", String.format("%.1f° -> %.1f°", heading, targetHeading));
            telemetry.addData("Error", String.format("%.1f°", error));
            telemetry.addData("Turn Power", String.format("%.2f", turn));
            telemetry.update();
        }
        drive.stopMotors();
        return "Stopped";
    }
}
