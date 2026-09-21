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
 * Lee el Limelight y entrega el CELL de nuestra alianza que mejor se ve.
 *
 * Los tags de BIOBUZZ se mueven con el HIVE, así que no sirven para ubicar al robot
 * en el campo. Aquí solo se usan para apuntar: ángulo al centro del CELL y distancia.
 */
public class CellVision {
    private final Limelight3A limelight;
    private final Alliance alliance;
    private final ClusterSolver solver = new ClusterSolver(VisionCfg.SLOPE_MEMORY_S);

    private long lastResultStampNs = -1;
    private Cell currentCell;

    // Para telemetría y para "Tune: Tiro"
    private CellTarget lastTarget;
    private long lastTargetNs;
    private double lastLatencyMs;
    private double lastTrigDistanceIn = Double.NaN;
    private double last3dDistanceIn = Double.NaN;

    public CellVision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        limelight = hardwareMap.get(Limelight3A.class, Names.LIMELIGHT);
        limelight.setPollRateHz(VisionCfg.POLL_RATE_HZ);
        limelight.pipelineSwitch(VisionCfg.APRILTAG_PIPELINE);
        limelight.start();   // sin esto getLatestResult() devuelve null
    }

    /**
     * Llamar una vez por ciclo.
     * @return el objetivo SOLO si llegó una foto nueva con tags de nuestra alianza.
     *         Devuelve null si no hay foto nueva o si no se ve ningún CELL nuestro.
     */
    public CellTarget update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        long stampNs = result.getControlHubTimeStampNanos();
        if (stampNs == lastResultStampNs) return null;   // la misma foto del ciclo pasado
        lastResultStampNs = stampNs;

        long nowNs = System.nanoTime();
        double pipelineMs = result.getCaptureLatency() + result.getTargetingLatency();
        long captureNs = stampNs - (long) (pipelineMs * 1e6);
        lastLatencyMs = (nowNs - captureNs) / 1e6;
        if (lastLatencyMs > VisionCfg.MAX_LATENCY_MS) return null;

        // Separa los tags por CELL, quedándose solo con los de nuestra alianza
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
                    fr.getTargetXDegrees(),
                    fr.getTargetYDegrees(),
                    fr.getTargetArea(),
                    horizontalDistance3d(fr.getTargetPoseCameraSpace())));
        }
        if (byCell.isEmpty()) return null;

        currentCell = chooseCell(byCell);
        List<TagSighting> tags = byCell.get(currentCell);

        ClusterSolver.Solution aim = solver.solve(currentCell, tags, nowNs);

        lastTrigDistanceIn = trigDistance(aim.txDeg, aim.tyDeg);
        last3dDistanceIn = mean3dDistance(tags);
        double distance = VisionCfg.USE_3D_DISTANCE ? last3dDistanceIn : lastTrigDistanceIn;

        lastTarget = new CellTarget(currentCell, aim.txDeg, aim.tyDeg, distance,
                tags.size(), aim.centered, captureNs);
        lastTargetNs = nowNs;
        return lastTarget;
    }

    /**
     * Se queda con el CELL que ya se venía siguiendo mientras siga a la vista, para que
     * la torreta no brinque entre los dos CELLS. Solo cambia si el otro muestra más tags.
     */
    private Cell chooseCell(Map<Cell, List<TagSighting>> byCell) {
        Cell best = null;
        int bestCount = 0;
        double bestArea = 0;
        for (Map.Entry<Cell, List<TagSighting>> e : byCell.entrySet()) {
            int count = e.getValue().size();
            double area = 0;
            for (TagSighting t : e.getValue()) area += t.area;
            if (count > bestCount || (count == bestCount && area > bestArea)) {
                best = e.getKey();
                bestCount = count;
                bestArea = area;
            }
        }
        List<TagSighting> current = currentCell == null ? null : byCell.get(currentCell);
        if (current != null && current.size() >= bestCount) return currentCell;
        return best;
    }

    /** Distancia horizontal con la altura conocida de los tags y el ángulo vertical. */
    private static double trigDistance(double txDeg, double tyDeg) {
        double elevationDeg = VisionCfg.CAMERA_PITCH_DEG + tyDeg;
        if (elevationDeg < 2.0) return Double.NaN;   // casi horizontal: la fórmula se dispara
        double forward = (VisionCfg.TAG_HEIGHT_IN - VisionCfg.CAMERA_HEIGHT_IN)
                / Math.tan(Math.toRadians(elevationDeg));
        // Si el objetivo no está centrado, la distancia real es un poco mayor
        return forward / Math.cos(Math.toRadians(txDeg));
    }

    /**
     * Distancia horizontal a un tag a partir de su pose en el espacio de la cámara
     * (x derecha, y abajo, z al frente), descontando la inclinación de la cámara.
     */
    private static double horizontalDistance3d(Pose3D pose) {
        if (pose == null) return Double.NaN;
        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
        if (p.x == 0 && p.y == 0 && p.z == 0) return Double.NaN;   // 3D apagado en el pipeline
        double pitch = Math.toRadians(VisionCfg.CAMERA_PITCH_DEG);
        double forward = p.z * Math.cos(pitch) + p.y * Math.sin(pitch);
        return Math.hypot(p.x, forward);
    }

    private static double mean3dDistance(List<TagSighting> tags) {
        double sum = 0;
        int n = 0;
        for (TagSighting t : tags) {
            if (!Double.isNaN(t.distanceIn)) {
                sum += t.distanceIn;
                n++;
            }
        }
        return n == 0 ? Double.NaN : sum / n;
    }

    /** Último objetivo visto, aunque no sea de este ciclo. Puede ser null. */
    public CellTarget getLastTarget() { return lastTarget; }

    public double secondsSinceTarget() {
        return lastTarget == null ? Double.POSITIVE_INFINITY : (System.nanoTime() - lastTargetNs) / 1e9;
    }

    public double getTrigDistanceIn() { return lastTrigDistanceIn; }
    public double get3dDistanceIn() { return last3dDistanceIn; }

    public void addTelemetry(Telemetry telemetry) {
        if (!limelight.isConnected()) {
            telemetry.addLine("!! LIMELIGHT DESCONECTADO");
            return;
        }
        if (lastTarget == null || secondsSinceTarget() > 0.5) {
            telemetry.addData("Vision", "sin objetivo (%s)", alliance);
            return;
        }
        telemetry.addData("Vision", "%s  tags %d/4%s", lastTarget.cell, lastTarget.tagCount,
                lastTarget.centered ? "" : "  SIN CENTRAR");
        telemetry.addData("  tx / ty", "%.1f / %.1f grados", lastTarget.txDeg, lastTarget.tyDeg);
        telemetry.addData("  distancia", "%.1f in   retraso %.0f ms", lastTarget.distanceIn, lastLatencyMs);
    }

    public void stop() {
        limelight.stop();
    }
}
