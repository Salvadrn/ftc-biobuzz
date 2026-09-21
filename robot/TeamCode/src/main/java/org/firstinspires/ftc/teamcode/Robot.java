package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Config.ImuCfg;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.vision.CellVision;

/** Junta todo el hardware del robot para que los OpModes no repitan la inicialización. */
public class Robot {
    public final MecanumDrive drive;
    public final Turret turret;
    public final Shooter shooter;
    public final CellVision vision;

    private final IMU imu;
    private double lastRawYawDeg = 0;
    private double continuousYawDeg = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        // Lee todos los sensores de cada hub en un solo viaje por ciclo: el loop corre más rápido
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive = new MecanumDrive(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        vision = new CellVision(hardwareMap, alliance);

        imu = ImuCfg.USE_IMU ? hardwareMap.tryGet(IMU.class, Names.IMU) : null;
        if (imu != null) {
            // CAMBIAR si el Control Hub no va con el logo hacia arriba y el USB hacia el frente
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            imu.resetYaw();
        }
    }

    /**
     * Rumbo del chasis en grados, positivo = izquierda, SIN el salto de 180 a -180.
     * Llamar una vez por ciclo. Devuelve 0 si no hay IMU.
     */
    public double updateYaw() {
        if (imu == null) return 0;
        double raw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double delta = raw - lastRawYawDeg;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        lastRawYawDeg = raw;
        continuousYawDeg += delta;
        return continuousYawDeg;
    }

    public boolean hasImu() { return imu != null; }

    public void stop() {
        drive.stop();
        shooter.stop();
        vision.stop();
    }
}
