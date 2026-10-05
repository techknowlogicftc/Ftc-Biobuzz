package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Four-motor mecanum drive.
 *
 * Hardware config names: "frontleft", "frontright", "rearleft", "rearright"
 */
public class MecanumDrive {

    private DcMotor frontLeft, frontRight, rearLeft, rearRight;

    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "frontleft");
        frontRight = hwMap.get(DcMotor.class, "frontright");
        rearLeft = hwMap.get(DcMotor.class, "rearleft1");
        rearRight = hwMap.get(DcMotor.class, "rearright");

        // Right side motors are reversed so positive power = forward on all wheels.
        // (Reversing the left side instead made the robot spin in place on forward.)
        // If the robot drives backward when the stick is pushed forward, flip all four.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, rearLeft, rearRight}) {
            // Direct power, no encoder feedback - motors with missing/loose encoder cables would stall in RUN_USING_ENCODER
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Stop immediately at zero power instead of coasting
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Drive the robot.
     *
     * @param forward positive = forward, negative = backward
     * @param strafe  positive = drift right, negative = drift left
     */
    public void drive(double forward, double strafe) {
        double frontLeftPower = forward + strafe;
        double frontRightPower = forward - strafe;
        double rearLeftPower = forward - strafe;
        double rearRightPower = forward + strafe;

        // Scale down so no wheel exceeds 1.0 while keeping the ratios (and direction) the same
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower))));

        frontLeft.setPower(frontLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        rearLeft.setPower(rearLeftPower / max);
        rearRight.setPower(rearRightPower / max);
    }

    /**
     * Run only the rear left motor (for testing wiring/config). Other motors are stopped.
     */
    public void runRearLeft(double power) {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(power);
        rearRight.setPower(0);
    }

    /**
     * Stop all four motors
     */
    public void stopMotors() {
        drive(0, 0);
    }
}
