import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.InterpTable;
import org.firstinspires.ftc.teamcode.vision.*;
import java.util.*;

public class Sim {
    static class FakeServo implements Servo {
        double pos = 0.5;
        public ServoController getController() { return null; }
        public int getPortNumber() { return 0; }
        public void setDirection(Direction d) {}
        public Direction getDirection() { return Direction.FORWARD; }
        public void setPosition(double p) { pos = p; }
        public double getPosition() { return pos; }
        public void scaleRange(double a, double b) {}
        public Manufacturer getManufacturer() { return Manufacturer.Other; }
        public String getDeviceName() { return "fake"; }
        public String getConnectionInfo() { return ""; }
        public int getVersion() { return 1; }
        public void resetDeviceConfigurationForOpMode() {}
        public void close() {}
    }

    // Proyecta la fila de tags: CELL a distancia d, rumbo relativo bearingDeg, fila girada skewDeg
    static List<TagSighting> project(double bearingDeg, double d, double skewDeg, boolean[] visible, Random rnd, boolean mirrored) {
        List<TagSighting> out = new ArrayList<>();
        double b = Math.toRadians(bearingDeg), sk = Math.toRadians(skewDeg);
        double cx = d * Math.sin(b), cz = d * Math.cos(b);      // x derecha, z al frente
        for (int i = 0; i < 4; i++) {
            if (!visible[i]) continue;
            double x = Cell.TAG_X_IN[i] * (mirrored ? -1 : 1);
            double px = cx + x * Math.cos(sk), pz = cz + x * Math.sin(sk);
            double tx = Math.toDegrees(Math.atan2(px, pz)) + rnd.nextGaussian() * 0.15;
            out.add(new TagSighting(i, tx, 10 + rnd.nextGaussian() * 0.15, 1.0, Double.NaN));
        }
        return out;
    }

    public static void main(String[] a) throws Exception {
        Random rnd = new Random(7);
        int fails = 0;

        // ---------- 1. InterpTable ----------
        InterpTable t = new InterpTable(new double[][]{{30, 2400}, {50, 2700}, {70, 3000}});
        fails += check("interp medio", t.get(40), 2550, 1e-9);
        fails += check("interp bajo el rango", t.get(10), 2400, 1e-9);
        fails += check("interp sobre el rango", t.get(99), 3000, 1e-9);

        // ---------- 2. ClusterSolver: el centro debe salir igual con tags tapados ----------
        boolean[][] masks = {{true,true,true,true},{true,true,false,false},{false,false,true,true},{true,false,false,true},{false,true,true,false}};
        for (boolean mirrored : new boolean[]{false, true}) {
            for (double skew : new double[]{0, 25, -35}) {
                for (boolean[] m : masks) {
                    ClusterSolver s = new ClusterSolver(1.0);
                    ClusterSolver.Solution sol = s.solve(Cell.RED_SCORING, project(8.0, 60, skew, m, new Random(1), mirrored), 0);
                    fails += check("centro skew=" + skew + " espejo=" + mirrored + " mask=" + Arrays.toString(m), sol.txDeg, 8.0, 1.0);
                }
            }
        }
        // promedio ingenuo con los dos tags de un lado, para comparar
        List<TagSighting> left = project(8.0, 60, 0, masks[1], new Random(1), false);
        double naive = (left.get(0).txDeg + left.get(1).txDeg) / 2;
        System.out.printf("   (referencia: promedio ingenuo con 2 tags de un lado = %.2f, real 8.00)%n", naive);

        // un solo tag: primero una foto completa, luego solo el tag de la orilla
        ClusterSolver s1 = new ClusterSolver(1.0);
        s1.solve(Cell.RED_SCORING, project(8.0, 60, 0, masks[0], rnd, false), 0);
        ClusterSolver.Solution one = s1.solve(Cell.RED_SCORING, project(8.0, 60, 0, new boolean[]{true,false,false,false}, rnd, false), 100_000_000L);
        fails += check("un tag con memoria", one.txDeg, 8.0, 1.0);
        fails += one.centered ? 0 : fail("un tag con memoria debe marcar centered");
        ClusterSolver.Solution stale = s1.solve(Cell.RED_SCORING, project(8.0, 60, 0, new boolean[]{true,false,false,false}, rnd, false), 5_000_000_000L);
        fails += !stale.centered ? 0 : fail("memoria vieja no debe usarse");
        ClusterSolver.Solution other = s1.solve(Cell.RED_AUDIENCE, project(8.0, 60, 0, new boolean[]{true,false,false,false}, rnd, false), 150_000_000L);
        fails += !other.centered ? 0 : fail("memoria de otro CELL no debe usarse");

        // ---------- 3. Torreta en lazo cerrado ----------
        for (double realSpeed : new double[]{250, 160, 400}) {
            fails += closedLoop(realSpeed, rnd);
        }

        System.out.println(fails == 0 ? "\nTODO OK" : "\nFALLAS: " + fails);
        System.exit(fails == 0 ? 0 : 1);
    }

    /** Servo real con velocidad distinta a la del modelo, cámara a 30 fps con 60 ms de retraso, chasis girando. */
    static int closedLoop(double realSpeedDegS, Random rnd) throws Exception {
        FakeServo servo = new FakeServo();
        Turret turret = new Turret(servo);
        ClusterSolver solver = new ClusterSolver(1.0);

        double trueTurret = 0, yaw = 0, targetField = 55;
        long start = System.nanoTime(), last = start, lastFrame = start;
        ArrayDeque<double[]> truth = new ArrayDeque<>();   // {tNs, rumbo real de la cámara}
        double worstAfterSettle = 0, maxOvershoot = 0;
        int reversals = 0; double prevCmd = 0, prevDir = 0;
        boolean lockedAtEnd = false;

        while (true) {
            long now = System.nanoTime();
            double tSec = (now - start) / 1e9, dt = (now - last) / 1e9;
            last = now;
            if (tSec > 4.0) break;

            // el chasis gira a 90 grados/s entre t=1.5 y t=2.5 (el driver rotando)
            if (tSec > 1.5 && tSec < 2.5) yaw += 90 * dt;

            double cmdAngle = Turret.servoToAngle(servo.pos);
            double step = realSpeedDegS * dt;
            trueTurret += Math.max(-step, Math.min(step, cmdAngle - trueTurret));
            truth.addLast(new double[]{now, yaw + trueTurret});
            while (truth.size() > 400) truth.removeFirst();

            CellTarget frame = null;
            if (now - lastFrame > 33_000_000L) {
                lastFrame = now;
                long captureNs = now - 60_000_000L;
                double headingThen = truth.peekFirst()[1];
                for (double[] smp : truth) { if (smp[0] <= captureNs) headingThen = smp[1]; else break; }
                double bearingRight = -(targetField - headingThen);   // tx: + = derecha
                if (Math.abs(bearingRight) < 27) {                    // dentro del FOV horizontal
                    boolean[] vis = {true, true, rnd.nextDouble() > 0.3, rnd.nextDouble() > 0.3};
                    ClusterSolver.Solution sol = solver.solve(Cell.RED_SCORING, project(bearingRight, 60, 15, vis, rnd, false), now);
                    frame = new CellTarget(Cell.RED_SCORING, sol.txDeg, sol.tyDeg, 60, 4, sol.centered, captureNs);
                }
            }
            // al inicio el objetivo está fuera de cámara: el operador lo acerca a mano
            double manual = (frame == null && !turret.hasTarget()) ? 1.0 : 0;
            turret.update(yaw, frame, true, manual);

            double err = (targetField - yaw) - trueTurret;
            double cmd = turret.getCommandedDeg();
            double dir = Math.signum(cmd - prevCmd);
            if (dir != 0) { if (prevDir != 0 && dir != prevDir && (tSec < 1.5 || tSec > 2.7)) reversals++; prevDir = dir; }
            prevCmd = cmd;
            if ((tSec > 1.0 && tSec < 1.5) || tSec > 3.2) worstAfterSettle = Math.max(worstAfterSettle, Math.abs(err));
            if (tSec > 1.5 && tSec < 2.5) maxOvershoot = Math.max(maxOvershoot, Math.abs(err));
            lockedAtEnd = turret.isLocked();
            Thread.sleep(8);
        }
        System.out.printf("lazo cerrado, servo real %.0f grados/s: error en reposo max %.2f, error durante giro del chasis max %.2f, cambios de sentido %d, locked=%b%n",
                realSpeedDegS, worstAfterSettle, maxOvershoot, reversals, lockedAtEnd);
        int f = 0;
        if (worstAfterSettle > 1.5) f += fail("no converge");
        if (!lockedAtEnd) f += fail("no reporta locked al final");
        if (reversals > 12) f += fail("oscila");
        return f;
    }

    static int check(String name, double got, double want, double tol) {
        boolean ok = Math.abs(got - want) <= tol;
        System.out.printf("%s %-70s %.3f (esperado %.3f)%n", ok ? "ok  " : "FALLA", name, got, want);
        return ok ? 0 : 1;
    }
    static int fail(String msg) { System.out.println("FALLA " + msg); return 1; }
}
