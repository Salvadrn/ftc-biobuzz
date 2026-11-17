package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Config.ImuCfg;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.SensorCfg;
import org.firstinspires.ftc.teamcode.feeder.BallInventory;
import org.firstinspires.ftc.teamcode.feeder.Feeder;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeF2;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;
import org.firstinspires.ftc.teamcode.sensors.RevColorReader;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.FileStateStore;
import org.firstinspires.ftc.teamcode.vision.CellVision;
import org.firstinspires.ftc.teamcode.vision.RivalNectarDetector;

/**
 * Todo el hardware del robot del CAD v99 en un solo lugar.
 *
 * 7 de 8 motores (4 de tracción, flywheel, M_F del intake y elevador) y 5 de 8 servos
 * (torreta, gap y ángulo del hood, fila y copa), como cuenta R503.
 */
public class Robot {
    public final Alliance alliance;
    public final MecanumDrive drive;
    public final Turret turret;
    public final Flywheel flywheel;
    public final Hood hood;
    public final Shooter shooter;
    public final Feeder feeder;
    public final Intake intake;
    public final CellVision vision;
    public final BallInventory inventory = new BallInventory();

    private final IMU imu;
    private boolean imuInitOk = false;
    private int imuFailedReads = 0;
    private double lastRawYawDeg = 0;
    private double continuousYawDeg = 0;
    private boolean initStarted = false;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        // Lee todos los sensores de cada hub en un solo viaje por ciclo
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive = new MecanumDrive(hardwareMap);
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap, new FileStateStore());
        shooter = new Shooter(flywheel, hood);

        feeder = new Feeder(
                hardwareMap.get(CRServo.class, Names.FILA_SERVO),
                hardwareMap.get(Servo.class, Names.COPA_SERVO),
                hardwareMap.get(DcMotor.class, Names.ELEVATOR),
                colorSensor(hardwareMap, Names.COLOR_UPPER, SensorCfg.UPPER),
                colorSensor(hardwareMap, Names.COLOR_COPA, SensorCfg.COPA),
                new BeamSensor(hardwareMap, Names.BEAM_FILA, SensorCfg.BEAM_ACTIVE_HIGH));

        // Para cambiar de intake: otra clase que implemente Intake, aquí.
        intake = new IntakeF2(
                hardwareMap.get(DcMotorEx.class, Names.INTAKE_MOTOR),
                new BeamSensor(hardwareMap, Names.MAGNET_F2, SensorCfg.MAGNET_ACTIVE_HIGH),
                new BeamSensor(hardwareMap, Names.BEAM_MOUTH, SensorCfg.BEAM_ACTIVE_HIGH),
                new BeamSensor(hardwareMap, Names.BEAM_THROAT, SensorCfg.BEAM_ACTIVE_HIGH),
                colorSensor(hardwareMap, Names.COLOR_THROAT, SensorCfg.THROAT),
                new RivalNectarDetector(hardwareMap, alliance),
                alliance);

        vision = new CellVision(hardwareMap, alliance);

        imu = ImuCfg.USE_IMU ? hardwareMap.tryGet(IMU.class, Names.IMU) : null;
        if (imu != null) {
            imuInitOk = imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(ImuCfg.LOGO, ImuCfg.USB)));
            imu.resetYaw();
        }
    }

    private static BallSensor colorSensor(HardwareMap hw, String name,
                                          org.firstinspires.ftc.teamcode.util.ColorClassifier.Thresholds t) {
        return new BallSensor(new RevColorReader(hw, name, SensorCfg.COLOR_GAIN), t, SensorCfg.COLOR_MIN_INTERVAL_S);
    }

    /**
     * Llamar en cada ciclo de INIT (R103.B): torreta al frente, hood a su casa, copa abajo
     * y suelta, F2 estacionado. Regresa true cuando ya quedó en la configuración de inicio.
     */
    public boolean initStep(long nowNs) {
        if (!initStarted) {
            initStarted = true;
            turret.startCentering(nowNs);
            hood.start(nowNs);
            feeder.initHardware(nowNs);
        }
        turret.update(nowNs, updateYaw(), null, false, 0);
        hood.update(nowNs);
        feeder.initUpdate(nowNs);
        boolean intakeReady = intake.initStep(nowNs);
        return intakeReady && turret.isPositionKnown() && hood.isReady(nowNs);
    }

    /**
     * Rumbo del chasis en grados, positivo = izquierda, SIN el salto de 180 a -180.
     * Una lectura fallida del IMU (el SDK la regresa con tiempo 0) se ignora.
     */
    public double updateYaw() {
        if (imu == null) return 0;
        YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
        if (a.getAcquisitionTime() == 0) {
            imuFailedReads++;
            return continuousYawDeg;
        }
        double raw = a.getYaw(AngleUnit.DEGREES);
        double delta = raw - lastRawYawDeg;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        lastRawYawDeg = raw;
        continuousYawDeg += delta;
        return continuousYawDeg;
    }

    /** Cuenta de pelotas (G407): entradas, rechazos y tiros. Llamar una vez por ciclo. */
    public void updateInventory() {
        for (int i = intake.consumeEntered(); i > 0; i--) inventory.entered();
        for (int i = intake.consumeRejected(); i > 0; i--) inventory.left();
        for (int i = feeder.consumeShots(); i > 0; i--) inventory.left();
        inventory.reconcile(feeder.sensedBalls() + (intake.hasBallAtThroat() ? 1 : 0));
    }

    public boolean hasImu() { return imu != null; }
    public boolean imuInitOk() { return imuInitOk; }
    public int imuFailedReads() { return imuFailedReads; }

    public void stop() {
        drive.stop();
        shooter.stop();
        feeder.stop();
        intake.stop();
        vision.stop();
    }
}
