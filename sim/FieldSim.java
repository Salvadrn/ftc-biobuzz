import com.qualcomm.hardware.limelightvision.LLFactory;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Config.VisionCfg;
import org.firstinspires.ftc.teamcode.vision.Cell;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.Random;

/**
 * Campo BIOBUZZ en 3D para la simulación, con la geometría del manual TU09:
 * pivote del HIVE a 43.95 in, inclinado 30°, filas de tags en la cara de abajo de cada
 * CELL (mirando hacia abajo y hacia el lado alto), a 14.27 in del pivote a lo largo del
 * HIVE y 1.39 in debajo de su eje. La CELL de arriba queda hacia +x.
 *
 * Marco del campo: x hacia el lado alto, y a lo largo del eje de giro del HIVE, z arriba.
 * La cámara es un Limelight 3A (54.5° x 42°, 640x480) que proyecta las 4 esquinas de cada
 * tag: el área "ta" sale de proyectar de verdad, no del modelo simplificado del robot.
 */
final class FieldSim {
    static final double PIVOT_H = 43.95, TILT_DEG = 30, ROW_S = 14.27, ROW_D = 1.39, TAG = 3.25;
    static final double HFOV = 54.5, VFOV = 42.0, W = 640, H = 480;
    static final double F_PX = (W / 2) / Math.tan(Math.toRadians(HFOV / 2));

    /** Centro de la fila de tags de la CELL de arriba (up = true) o de abajo. */
    static double[] rowCenter(boolean up) {
        double t = Math.toRadians(TILT_DEG);
        double s = up ? ROW_S : -ROW_S;
        return new double[]{s * Math.cos(t) + ROW_D * Math.sin(t), 0, PIVOT_H + s * Math.sin(t) - ROW_D * Math.cos(t)};
    }

    static double[] normal() {
        double t = Math.toRadians(TILT_DEG);
        return new double[]{Math.sin(t), 0, -Math.cos(t)};
    }

    static double[] alongHive() {
        double t = Math.toRadians(TILT_DEG);
        return new double[]{Math.cos(t), 0, Math.sin(t)};
    }

    /** Pose de la cámara: eje de la torreta en (ax, ay) del piso, rumbo de la torreta en el campo. */
    static final class Cam {
        final double x, y, z, yaw, pitch;
        Cam(double axisX, double axisY, double turretHeadingRad, double behindAxis, double height, double pitchDeg) {
            this.x = axisX - behindAxis * Math.cos(turretHeadingRad);
            this.y = axisY - behindAxis * Math.sin(turretHeadingRad);
            this.z = height;
            this.yaw = turretHeadingRad;
            this.pitch = Math.toRadians(pitchDeg);
        }

        /** Punto del campo en coordenadas de cámara del Limelight: {x der, y abajo, z al frente}. */
        double[] toCam(double[] w) {
            double dx = w[0] - x, dy = w[1] - y, dz = w[2] - z;
            double f = dx * Math.cos(yaw) + dy * Math.sin(yaw);
            double l = -dx * Math.sin(yaw) + dy * Math.cos(yaw);
            double camZ = f * Math.cos(pitch) + dz * Math.sin(pitch);
            double camY = f * Math.sin(pitch) - dz * Math.cos(pitch);
            return new double[]{-l, camY, camZ};
        }
    }

    /** Un tag visto: tx, ty (sin crosshair), área en %, posición en cámara (in). */
    static final class Seen {
        int id;
        double tx, ty, ta;
        double[] cam;
    }

    /** Proyecta un tag; null si no se vería (fuera de cuadro, de espaldas o muy chico). */
    static Seen project(Cam c, double[] center, int id, Random rnd, double angNoise, double areaNoise) {
        double[] n = normal();
        double[] toCam = {c.x - center[0], c.y - center[1], c.z - center[2]};
        double dist = Math.sqrt(toCam[0] * toCam[0] + toCam[1] * toCam[1] + toCam[2] * toCam[2]);
        double cosView = (n[0] * toCam[0] + n[1] * toCam[1] + n[2] * toCam[2]) / dist;
        if (cosView < Math.cos(Math.toRadians(70))) return null;

        double[] p = c.toCam(center);
        if (p[2] <= 1) return null;
        double tx = Math.toDegrees(Math.atan(p[0] / p[2]));
        double ty = Math.toDegrees(Math.atan(-p[1] / p[2]));
        if (Math.abs(tx) > HFOV / 2 || Math.abs(ty) > VFOV / 2) return null;

        // Área: proyectar las 4 esquinas y usar la fórmula del cordón
        double[] u = alongHive();
        double h = TAG / 2;
        double[][] corners = {
                {center[0] - h * u[0], center[1] - h, center[2] - h * u[2]},
                {center[0] - h * u[0], center[1] + h, center[2] - h * u[2]},
                {center[0] + h * u[0], center[1] + h, center[2] + h * u[2]},
                {center[0] + h * u[0], center[1] - h, center[2] + h * u[2]}};
        double area = 0;
        double[][] px = new double[4][];
        for (int i = 0; i < 4; i++) {
            double[] q = c.toCam(corners[i]);
            if (q[2] <= 1) return null;
            px[i] = new double[]{F_PX * q[0] / q[2], F_PX * q[1] / q[2]};
        }
        for (int i = 0; i < 4; i++) {
            double[] a = px[i], b = px[(i + 1) % 4];
            area += a[0] * b[1] - b[0] * a[1];
        }
        double ta = Math.abs(area) / 2 / (W * H) * 100;
        if (ta < 0.03) return null;   // demasiado chico para detectarlo

        Seen s = new Seen();
        s.id = id;
        s.tx = tx + rnd.nextGaussian() * angNoise;
        s.ty = ty + rnd.nextGaussian() * angNoise;
        s.ta = ta * (1 + rnd.nextGaussian() * areaNoise);
        s.cam = p;
        return s;
    }

    /**
     * Arma un LLResult real del SDK con lo que vería la cámara.
     * @param upCell CELL que está arriba; su compañera es la de abajo
     */
    static LLResult result(Cam c, Cell upCell, double tsMs, long stalenessMs, boolean full3d,
                           Random rnd, double angNoise, double areaNoise) {
        JSONArray fids = new JSONArray();
        Cell[] cells = {upCell, upCell.partner()};
        for (int k = 0; k < 2; k++) {
            double[] rc = rowCenter(k == 0);
            for (int i = 0; i < 4; i++) {
                double[] center = {rc[0], rc[1] + Cell.TAG_X_IN[i], rc[2]};
                Seen s = project(c, center, cells[k].firstTagId + i, rnd, angNoise, areaNoise);
                if (s == null) continue;
                JSONObject f = new JSONObject();
                f.put("fID", s.id);
                f.put("fam", "36H11");
                f.put("tx", s.tx);
                f.put("ty", s.ty);
                f.put("tx_nocross", s.tx);
                f.put("ty_nocross", s.ty);
                f.put("ta", s.ta);
                JSONArray pose = new JSONArray();
                double m = 0.0254;
                if (full3d) {
                    pose.put(s.cam[0] * m).put(s.cam[1] * m).put(s.cam[2] * m).put(0).put(0).put(0);
                } else {
                    pose.put(0).put(0).put(0).put(0).put(0).put(0);
                }
                f.put("t6t_cs", pose);
                fids.put(f);
            }
        }
        JSONObject j = new JSONObject();
        j.put("v", fids.length() > 0 ? 1 : 0);
        j.put("ts", tsMs);
        j.put("cl", 20.0);
        j.put("tl", 15.0);
        j.put("pID", VisionCfg.APRILTAG_PIPELINE);
        j.put("Fiducial", fids);
        return LLFactory.make(j, stalenessMs);
    }

    /** Rumbo verdadero (grados, + izquierda) y distancia horizontal del eje de la torreta a la fila de arriba. */
    static double[] truthFromAxis(double axisX, double axisY, double turretHeadingRad) {
        double[] rc = rowCenter(true);
        return truthTo(rc[0], rc[1], axisX, axisY, turretHeadingRad);
    }

    /**
     * Lo mismo, pero al centro de la abertura de la CELL de arriba: APERTURE_OUT_IN hacia
     * afuera de la fila (la fila corre en y, así que su normal horizontal es x), del lado
     * de la cámara. Ahí apunta la visión cuando ve 2 o más tags.
     */
    static double[] truthApertureFromAxis(double axisX, double axisY, double turretHeadingRad) {
        double[] rc = rowCenter(true);
        double side = Math.signum(axisX - rc[0]);
        return truthTo(rc[0] + side * VisionCfg.APERTURE_OUT_IN, rc[1], axisX, axisY, turretHeadingRad);
    }

    private static double[] truthTo(double px, double py, double axisX, double axisY, double turretHeadingRad) {
        double[] rc = {px, py};
        double dx = rc[0] - axisX, dy = rc[1] - axisY;
        double bearingField = Math.atan2(dy, dx);
        double rel = Math.toDegrees(bearingField - turretHeadingRad);
        while (rel > 180) rel -= 360;
        while (rel <= -180) rel += 360;
        return new double[]{rel, Math.hypot(dx, dy), Math.toDegrees(bearingField)};
    }
}
