package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.VisionCfg;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * Lee el Limelight y entrega la CELL de ARRIBA de nuestra alianza, vista desde el eje
 * de la torreta.
 *
 * Tres cosas que aprendimos en la auditoría del 27-oct y aquí ya se respetan:
 *  1. getControlHubTimeStampNanos() del SDK es hora de reloj (ms x 1e6), no la base de
 *     System.nanoTime(). El instante de la foto se calcula con getStaleness() y las
 *     latencias del Limelight.
 *  2. El SDK crea un resultado nuevo en cada consulta (100 Hz) aunque la cámara no haya
 *     tomado otra foto (≤ 50 fps con tags). La foto se reconoce por getTimestamp(), el
 *     reloj del propio Limelight.
 *  3. Desde donde se tira se ven las dos CELLS de la alianza. HiveGeometry y HiveBelief
 *     deciden cuál es la de arriba.
 *
 * Los tags de BIOBUZZ se mueven con el HIVE: no sirven para ubicar al robot en el campo.
 */
public class CellVision {
    private final Limelight3A limelight;    // null en la simulación
    private final Alliance alliance;
    private final ClusterSolver solver = new ClusterSolver(VisionCfg.SLOPE_MEMORY_S);
    private final HiveBelief belief;

    private double lastLlTimestamp = Double.NaN;
    private long lastPipelineFixNs = 0;

    // Para telemetría y para "Tune: Tiro"
    private CellTarget lastTarget;
    private long lastTargetNs;
    private double lastLatencyMs = Double.NaN;
    private double last3dDistanceIn = Double.NaN;
    private double lastSizeScore = Double.NaN;
    private String status = "sin datos";
    private boolean wrongPipeline = false;

    public CellVision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        this.belief = new HiveBelief(alliance, VisionCfg.UPDOWN_CONFIRM_FRAMES);
        limelight = hardwareMap.get(Limelight3A.class, Names.LIMELIGHT);
        limelight.setPollRateHz(VisionCfg.POLL_RATE_HZ);   // antes de start(): con el Limelight corriendo se ignora
        limelight.pipelineSwitch(VisionCfg.APRILTAG_PIPELINE);
        limelight.start();
    }

    /** Para la simulación: sin hardware, se alimenta con process(). */
    public CellVision(Alliance alliance) {
        this.alliance = alliance;
        this.belief = new HiveBelief(alliance, VisionCfg.UPDOWN_CONFIRM_FRAMES);
        this.limelight = null;
    }

    /**
     * Llamar una vez por ciclo.
     * @return el objetivo SOLO si llegó una foto nueva con la CELL de arriba a la vista.
     */
    public CellTarget update(long nowNs) {
        if (limelight == null) return null;
        LLResult result = limelight.getLatestResult();
        CellTarget target = process(result, nowNs);
        if (wrongPipeline && nowNs - lastPipelineFixNs > 3_000_000_000L) {
            lastPipelineFixNs = nowNs;
            limelight.pipelineSwitch(VisionCfg.APRILTAG_PIPELINE);
        }
        return target;
    }

    /** Toda la lógica, sin hardware. Público para la simulación. */
    public CellTarget process(LLResult result, long nowNs) {
        if (result == null) return null;

        double ts = result.getTimestamp();
        if (ts == lastLlTimestamp) return null;          // la misma foto del Limelight
        lastLlTimestamp = ts;

        wrongPipeline = result.getPipelineIndex() != VisionCfg.APRILTAG_PIPELINE;
        if (!result.isValid()) {
            status = "no se ven tags";
            return null;
        }

        double ageMs = Math.max(0, result.getStaleness())
                + result.getCaptureLatency() + result.getTargetingLatency();
        lastLatencyMs = ageMs;
        if (ageMs > VisionCfg.MAX_LATENCY_MS) {
            status = "foto vieja";
            return null;
        }
        long captureNs = nowNs - (long) (ageMs * 1e6);

        // Separa los tags por CELL, solo los de nuestra alianza
        Map<Cell, List<TagSighting>> byCell = new EnumMap<>(Cell.class);
        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            Cell cell = Cell.fromTagId(fr.getFiducialId());
            if (cell == null || cell.alliance != alliance) continue;
            List<TagSighting> list = byCell.get(cell);
            if (list == null) {
                list = new ArrayList<>();
                byCell.put(cell, list);
            }
            list.add(new TagSighting(
                    cell.memberIndex(fr.getFiducialId()),
                    fr.getTargetXDegreesNoCrosshair(),
                    fr.getTargetYDegreesNoCrosshair(),
                    fr.getTargetArea(),
                    tagHeight3d(fr.getTargetPoseCameraSpace())));
        }
        if (byCell.isEmpty()) {
            status = "no se ven tags de " + alliance;
            return null;
        }

        belief.addEvidence(claimUpCell(byCell), lastReason);

        Cell up = belief.upCell();
        if (up == null) {
            status = "no se sabe cuál CELL está arriba";
            return null;
        }
        List<TagSighting> tags = byCell.get(up);
        if (tags == null) {
            status = "solo se ve la CELL de abajo";
            return null;
        }

        ClusterSolver.Solution aim = solver.solve(up, tags, nowNs);
        HiveGeometry.Ray ray = HiveGeometry.ray(aim.txDeg, aim.tyDeg, VisionCfg.CAMERA_PITCH_DEG);
        double camDist = HiveGeometry.horizontalDistance(ray,
                VisionCfg.TAG_ROW_HEIGHT_UP_IN - VisionCfg.CAMERA_HEIGHT_IN);

        last3dDistanceIn = mean3dDistance(result, up);
        if (VisionCfg.USE_3D_DISTANCE && !Double.isNaN(last3dDistanceIn)) {
            camDist = last3dDistanceIn;
        }
        if (Double.isNaN(camDist)) {
            status = "la fila quedó bajo el horizonte de la cámara";
            return null;
        }

        double[] axis = HiveGeometry.fromCameraToTurretAxis(ray.azimuthLeftDeg, camDist,
                VisionCfg.CAMERA_BEHIND_AXIS_IN);
        double bearing = axis[0] + VisionCfg.AIM_TRIM_DEG;

        status = "ok";
        lastTarget = new CellTarget(up, bearing, axis[1], tags.size(), aim.centered, captureNs);
        lastTargetNs = nowNs;
        return lastTarget;
    }

    private String lastReason = "";

    /**
     * Decide con esta foto cuál CELL está arriba. Devuelve null si la foto no alcanza.
     *
     *  - Si se ven las dos filas, la de arriba se ve más alta: basta comparar elevaciones
     *    (si la diferencia es clara).
     *  - Con una sola fila: altura por la pose 3D (si está activada) o prueba de tamaño.
     */
    private Cell claimUpCell(Map<Cell, List<TagSighting>> byCell) {
        Cell[] both = Cell.ofAlliance(alliance);
        List<TagSighting> a = byCell.get(both[0]);
        List<TagSighting> b = byCell.get(both[1]);

        if (a != null && b != null) {
            double elevA = meanRay(a).elevationDeg;
            double elevB = meanRay(b).elevationDeg;
            if (Math.abs(elevA - elevB) >= VisionCfg.UPDOWN_ELEVATION_SPLIT_DEG) {
                lastReason = "dos filas";
                return elevA > elevB ? both[0] : both[1];
            }
        }

        Cell claim = null;
        for (Cell cell : both) {
            List<TagSighting> tags = byCell.get(cell);
            if (tags == null) continue;
            Boolean isUp = classifySingle(tags);
            if (isUp == null) continue;
            Cell c = isUp ? cell : cell.partner();
            if (claim != null && claim != c) {
                lastReason = "fotos contradictorias";
                return null;
            }
            claim = c;
        }
        return claim;
    }

    /** true = fila de arriba, false = fila de abajo, null = no se puede decir. */
    private Boolean classifySingle(List<TagSighting> tags) {
        double midHeight = 0.5 * (VisionCfg.TAG_ROW_HEIGHT_UP_IN + VisionCfg.TAG_ROW_HEIGHT_DOWN_IN);

        if (VisionCfg.USE_3D_FOR_UPDOWN) {
            double sum = 0;
            int n = 0;
            for (TagSighting t : tags) {
                if (!Double.isNaN(t.height3dIn)) {
                    sum += t.height3dIn;
                    n++;
                }
            }
            if (n > 0) {
                double h = sum / n;
                if (Math.abs(h - midHeight) >= VisionCfg.UPDOWN_3D_MARGIN_IN) {
                    lastReason = String.format("altura 3D %.0f in", h);
                    return h > midHeight;
                }
            }
        }

        double scoreSum = 0;
        int n = 0;
        for (TagSighting t : tags) {
            HiveGeometry.Ray r = HiveGeometry.ray(t.txDeg, t.tyDeg, VisionCfg.CAMERA_PITCH_DEG);
            double s = HiveGeometry.upDownSizeScore(r, t.areaPct, VisionCfg.CAMERA_HEIGHT_IN,
                    VisionCfg.TAG_ROW_HEIGHT_UP_IN, VisionCfg.TAG_ROW_HEIGHT_DOWN_IN,
                    VisionCfg.TAG_AREA_K, VisionCfg.TAG_COS_VIEW);
            if (!Double.isNaN(s)) {
                scoreSum += s;
                n++;
            }
        }
        if (n == 0) return null;
        lastSizeScore = scoreSum / n;
        if (Math.abs(lastSizeScore) < VisionCfg.UPDOWN_SIZE_MARGIN) return null;
        lastReason = String.format("tamaño %+.2f", lastSizeScore);
        return lastSizeScore < 0;
    }

    private static HiveGeometry.Ray meanRay(List<TagSighting> tags) {
        double tx = 0, ty = 0;
        for (TagSighting t : tags) {
            tx += t.txDeg;
            ty += t.tyDeg;
        }
        return HiveGeometry.ray(tx / tags.size(), ty / tags.size(), VisionCfg.CAMERA_PITCH_DEG);
    }

    /** Altura de un tag por la pose 3D del Limelight, o NaN si "Full 3D" está apagado. */
    private static double tagHeight3d(Pose3D pose) {
        if (pose == null) return Double.NaN;
        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
        if (p.x == 0 && p.y == 0 && p.z == 0) return Double.NaN;
        return HiveGeometry.heightFromCameraSpace(p.y, p.z, VisionCfg.CAMERA_HEIGHT_IN, VisionCfg.CAMERA_PITCH_DEG);
    }

    /** Distancia horizontal promedio de la cámara a los tags de una CELL por la pose 3D. */
    private static double mean3dDistance(LLResult result, Cell cell) {
        double sum = 0;
        int n = 0;
        double p = Math.toRadians(VisionCfg.CAMERA_PITCH_DEG);
        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            if (Cell.fromTagId(fr.getFiducialId()) != cell) continue;
            Pose3D pose = fr.getTargetPoseCameraSpace();
            if (pose == null) continue;
            Position pos = pose.getPosition().toUnit(DistanceUnit.INCH);
            if (pos.x == 0 && pos.y == 0 && pos.z == 0) continue;
            double forward = pos.z * Math.cos(p) + pos.y * Math.sin(p);
            sum += Math.hypot(pos.x, forward);
            n++;
        }
        return n == 0 ? Double.NaN : sum / n;
    }

    // ---------- piloto ----------

    /** El piloto avisa que el HIVE se volteó (o que la visión eligió mal). */
    public void flipHive() {
        belief.flip();
        solver.reset();
    }

    public Cell upCell() {
        return belief.upCell();
    }

    // ---------- lectura ----------

    /** Último objetivo visto, aunque no sea de este ciclo. Puede ser null. */
    public CellTarget getLastTarget() { return lastTarget; }

    public double secondsSinceTarget(long nowNs) {
        return lastTarget == null ? Double.POSITIVE_INFINITY : (nowNs - lastTargetNs) / 1e9;
    }

    public double get3dDistanceIn() { return last3dDistanceIn; }
    public double getSizeScore() { return lastSizeScore; }
    public String getStatus() { return status; }

    public void addTelemetry(Telemetry telemetry, long nowNs) {
        if (limelight != null && !limelight.isConnected()) {
            telemetry.addLine("!! LIMELIGHT DESCONECTADO");
            return;
        }
        if (wrongPipeline) telemetry.addLine("!! El Limelight no está en el pipeline de AprilTags");
        Cell up = belief.upCell();
        telemetry.addData("HIVE", "arriba: %s (%s)", up == null ? "?" : up, belief.reason());
        if (lastTarget == null || secondsSinceTarget(nowNs) > 0.5) {
            telemetry.addData("Visión", status);
            return;
        }
        telemetry.addData("Visión", "%s  tags %d/4%s", lastTarget.cell, lastTarget.tagCount,
                lastTarget.centered ? "" : "  SIN CENTRAR");
        telemetry.addData("  rumbo / dist", "%.1f° / %.1f in   retraso %.0f ms",
                lastTarget.bearingDeg, lastTarget.distanceIn, lastLatencyMs);
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
