package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config.Names;

/** Chasis mecanum básico, centrado en el robot. Si una rueda gira al revés, cambiar su dirección aquí. */
public class MecanumDrive {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, Names.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, Names.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, Names.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, Names.BACK_RIGHT);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * @param forward + = adelante
     * @param strafe  + = a la derecha
     * @param turn    + = girar a la derecha
     */
    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Si alguna potencia pasa de 1, se escalan todas para conservar la dirección
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    public void stop() {
        drive(0, 0, 0);
    }
}
