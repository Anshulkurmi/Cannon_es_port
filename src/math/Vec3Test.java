package math;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class Vec3Test {

    @Test
    public void testConstruct() {
        Vec3 v = new Vec3(1, 2, 3);
        assertEquals(1, v.x, 1e-6);
        assertEquals(2, v.y, 1e-6);
        assertEquals(3, v.z, 1e-6);
    }

    @Test
    public void testCross() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = new Vec3(4, 5, 6);

        Vec3 v_x_u = v.cross(u);
        assertEquals(-3, v_x_u.x, 1e-6);
        assertEquals(6, v_x_u.y, 1e-6);
        assertEquals(-3, v_x_u.z, 1e-6);
    }

    @Test
    public void testSet() {
        Vec3 v = new Vec3(1, 2, 3);
        v.set(4, 5, 6);

        assertEquals(4, v.x, 1e-6);
        assertEquals(5, v.y, 1e-6);
        assertEquals(6, v.z, 1e-6);
    }

    @Test
    public void testSetZero() {
        Vec3 v = new Vec3(1, 2, 3);
        v.setZero();
        assertEquals(0, v.x, 1e-6);
        assertEquals(0, v.y, 1e-6);
        assertEquals(0, v.z, 1e-6);
    }

    @Test
    public void testVadd() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = new Vec3(4, 5, 6);

        Vec3 w = v.vadd(u);
        assertEquals(5, w.x, 1e-6);
        assertEquals(7, w.y, 1e-6);
        assertEquals(9, w.z, 1e-6);
    }

    @Test
    public void testVsub() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = new Vec3(4, 6, 8);

        Vec3 w = v.vsub(u);
        assertEquals(-3, w.x, 1e-6);
        assertEquals(-4, w.y, 1e-6);
        assertEquals(-5, w.z, 1e-6);
    }

    @Test
    public void testNormalize() {
        Vec3 v = new Vec3(2, 3, 6);
        double norm = v.normalize();
        assertEquals(7, norm, 1e-6);
        assertEquals(2 / 7.0, v.x, 1e-6);
        assertEquals(3 / 7.0, v.y, 1e-6);
        assertEquals(6 / 7.0, v.z, 1e-6);
    }

    @Test
    public void testNormalizeZero() {
        Vec3 v = new Vec3(0, 0, 0);
        double something = v.normalize();
        assertTrue(Double.isNaN(something));
    }

    @Test
    public void testUnit() {
        Vec3 v = new Vec3(2, 3, 6);

        // Test for returning a new vector
        Vec3 unit_v = v.unit();
        assertEquals(2 / 7.0, unit_v.x, 1e-6);
        assertEquals(3 / 7.0, unit_v.y, 1e-6);
        assertEquals(6 / 7.0, unit_v.z, 1e-6);
    }

    @Test
    public void testUnitZero() {
        Vec3 v = new Vec3(0, 0, 0);
        Vec3 something = v.unit();
        assertNotNull(something);
    }

    @Test
    public void testLength() {
        Vec3 v = new Vec3(2, 3, 6);
        double v_len = v.length();
        assertEquals(7, v_len, 1e-6);
    }

    @Test
    public void testLengthSquared() {
        Vec3 v = new Vec3(2, 3, 6);
        double v_len_2 = v.lengthSquared();
        assertEquals(49, v_len_2, 1e-6);
    }

    @Test
    public void testDistanceTo() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = new Vec3(3, 5, 9);
        assertEquals(7, v.distanceTo(u), 1e-6);
        assertEquals(7, u.distanceTo(v), 1e-6);
    }

    @Test
    public void testDistanceSquared() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = new Vec3(3, 5, 9);
        assertEquals(49, v.distanceSquared(u), 1e-6);
        assertEquals(49, u.distanceSquared(v), 1e-6);
    }

    @Test
    public void testScale() {
        Vec3 v = new Vec3(1, 2, 3);

        Vec3 v2 = v.scale(2);
        assertEquals(2, v2.x, 1e-6);
        assertEquals(4, v2.y, 1e-6);
        assertEquals(6, v2.z, 1e-6);
    }

    @Test
    public void testVmul() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = new Vec3(4, 5, 6);

        Vec3 vu = v.vmul(u);
        assertEquals(4, vu.x, 1e-6);
        assertEquals(10, vu.y, 1e-6);
        assertEquals(18, vu.z, 1e-6);
    }

    @Test
    public void testAddScaledVector() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = new Vec3(4, 5, 6);

        Vec3 v3u = v.addScaledVector(3, u);
        assertEquals(13, v3u.x, 1e-6);
        assertEquals(17, v3u.y, 1e-6);
        assertEquals(21, v3u.z, 1e-6);
    }

    @Test
    public void testDot() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = new Vec3(4, 5, 6);
        double dot = v.dot(u);

        assertEquals(4 + 10 + 18, dot, 1e-6);

        v = new Vec3(3, 2, 1);
        u = new Vec3(4, 5, 6);
        dot = v.dot(u);

        assertEquals(12 + 10 + 6, dot, 1e-6);
    }

    @Test
    public void testIsZero() {
        Vec3 v = new Vec3(1, 2, 3);
        assertFalse(v.isZero());

        Vec3 u = new Vec3(0, 0, 0);
        assertTrue(u.isZero());
    }

    @Test
    public void testNegate() {
        Vec3 v = new Vec3(1, 2, 3);

        Vec3 neg_v = v.negate();
        assertEquals(-1, neg_v.x, 1e-6);
        assertEquals(-2, neg_v.y, 1e-6);
        assertEquals(-3, neg_v.z, 1e-6);
    }

    @Test
    public void testTangents() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 vt1 = new Vec3();
        Vec3 vt2 = new Vec3();
        v.tangents(vt1, vt2);
        assertEquals(0, vt1.dot(v), 1e-6);
        assertEquals(0, vt2.dot(v), 1e-6);
        assertEquals(0, vt1.dot(vt2), 1e-6);
    }

    @Test
    public void testToString() {
        Vec3 v = new Vec3(1, 2, 3);
        assertEquals("1.0,2.0,3.0", v.toString());
    }

    @Test
    public void testToArray() {
        Vec3 v = new Vec3(1, 2, 3);
        double[] v_a = v.toArray();
        assertEquals(1, v_a[0], 1e-6);
        assertEquals(2, v_a[1], 1e-6);
        assertEquals(3, v_a[2], 1e-6);
    }

    @Test
    public void testCopy() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = new Vec3();
        u.copy(v);
        assertEquals(1, u.x, 1e-6);
        assertEquals(2, u.y, 1e-6);
        assertEquals(3, u.z, 1e-6);
    }

    @Test
    public void testLerp() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = new Vec3(5, 6, 7);

        Vec3 vlerpu = new Vec3();
        v.lerp(u, 0.5, vlerpu);
        assertEquals(3, vlerpu.x, 1e-6);
        assertEquals(4, vlerpu.y, 1e-6);
        assertEquals(5, vlerpu.z, 1e-6);
    }

    @Test
    public void testAlmostEquals() {
        assertTrue(new Vec3(1, 0, 0).almostEquals(new Vec3(1, 0, 0), 1e-6));
        assertFalse(new Vec3(1e-5, 1e-5, 1e-5).almostEquals(new Vec3(), 1e-6));
        assertTrue(new Vec3(1e-7, 1e-7, 1e-7).almostEquals(new Vec3(), 1e-6));
    }

    @Test
    public void testAlmostZero() {
        assertFalse(new Vec3(1, 0, 0).almostZero());
        assertTrue(new Vec3(1e-7, 1e-7, 1e-7).almostZero(1e-6));
        assertFalse(new Vec3(1e-5, 1e-5, 1e-5).almostZero(1e-6));
    }

    @Test
    public void testIsAntiparallelTo() {
        assertTrue(new Vec3(1, 0, 0).isAntiparallelTo(new Vec3(-1, 0, 0), 1e-6));
        assertFalse(new Vec3(1, 0, 0).isAntiparallelTo(new Vec3(1, 0, 0), 1e-6));
    }

    @Test
    public void testClone() {
        Vec3 v = new Vec3(1, 2, 3);
        Vec3 u = v.clone();
        v.x = 4;
        v.y = 5;
        v.z = 6;
        assertEquals(1, u.x, 1e-6);
        assertEquals(2, u.y, 1e-6);
        assertEquals(3, u.z, 1e-6);
    }
    
    
}
