package math;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
 
public class QuaternionTest {

    @Test
    public void testConstructor() {
        Quaternion q = new Quaternion(1, 2, 3, 4);
        assertEquals(1, q.x, 0.001);
        assertEquals(2, q.y, 0.001);
        assertEquals(3, q.z, 0.001);
        assertEquals(4, q.w, 0.001);
    }

    @Test
    public void testConjugate() {
        Quaternion q = new Quaternion(1, 2, 3, 4);
        q.conjugate(q);
        assertEquals(-1, q.x, 0.001);
        assertEquals(-2, q.y, 0.001);
        assertEquals(-3, q.z, 0.001);
        assertEquals(4, q.w, 0.001);
    }

    @Test
    public void testInverse() {
        Quaternion q = new Quaternion(1, 2, 3, 4);
        double denominator = 1 * 1 + 2 * 2 + 3 * 3 + 4 * 4;
        q.inverse(q);

        // Quaternion inverse is conjugate(q) / ||q||^2
        assertEquals(-1 / denominator, q.x, 0.001);
        assertEquals(-2 / denominator, q.y, 0.001);
        assertEquals(-3 / denominator, q.z, 0.001);
        assertEquals(4 / denominator, q.w, 0.001);
    }

    @Test
    public void testToEuler() {
        Quaternion q = new Quaternion();
        q.setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 4);
        Vec3 euler = new Vec3();
        q.toEuler(euler,"YZX");

        // We should expect (0, 0, pi/4)
        assertEquals(0, euler.x, 0.001);
        assertEquals(0, euler.y, 0.001);
        assertEquals(Math.PI / 4, euler.z, 0.001);
    }

    @Test
    public void testSetFromVectors() {
        Quaternion q = new Quaternion();
        q.setFromVectors(new Vec3(1, 0, 0), new Vec3(-1, 0, 0));
        assertTrue(q.vmult(new Vec3(1, 0, 0)).almostEquals(new Vec3(-1, 0, 0)));

        q.setFromVectors(new Vec3(0, 1, 0), new Vec3(0, -1, 0));
        assertTrue(q.vmult(new Vec3(0, 1, 0)).almostEquals(new Vec3(0, -1, 0)));

        q.setFromVectors(new Vec3(0, 0, 1), new Vec3(0, 0, -1));
        assertTrue(q.vmult(new Vec3(0, 0, 1)).almostEquals(new Vec3(0, 0, -1)));

        q.setFromVectors(new Vec3(1, 2, 3), new Vec3(2, 1, 3));
        assertTrue(q.vmult(new Vec3(1, 2, 3)).almostEquals(new Vec3(2, 1, 3)));
    }

    @Test
    public void testSlerp() {
        Quaternion qa = new Quaternion();
        Quaternion qb = new Quaternion();
        qa.slerp(qb, 0.5);
        assertEquals(qa, qb);

        qa.setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 4);
        qb.setFromAxisAngle(new Vec3(0, 0, 1), -Math.PI / 4);
        qa.slerp(qb, 0.5);
        assertEquals(qb, new Quaternion());
    }

    @Test
    public void testSet() {
        Quaternion q = new Quaternion(1, 2, 3, 4);
        q.set(5, 6, 7, 8);
        assertEquals(5, q.x, 0.001);
        assertEquals(6, q.y, 0.001);
        assertEquals(7, q.z, 0.001);
        assertEquals(8, q.w, 0.001);
    }

    @Test
    public void testToString() {
        Quaternion q = new Quaternion(1, 2, 3, 4);
        assertEquals("1.0,2.0,3.0,4.0", q.toString());
    }

    @Test
    public void testToArray {
        Quaternion q = new Quaternion(1, 2, 3, 4);
        double[] qa = q.toArray;
        assertEquals(1, qa[0], 0.001);
        assertEquals(2, qa[1], 0.001);
        assertEquals(3, qa[2], 0.001);
        assertEquals(4, qa[3], 0.001);
    }

    @Test
    public void testCopy {
        Quaternion q = new Quaternion(1, 2, 3, 4);
        Quaternion qc = new Quaternion();
        qc.copy(q);
        q.set(4, 5, 6, 7);
        assertEquals(1, qc.x, 0.001);
        assertEquals(2, qc.y, 0.001);
        assertEquals(3, qc.z, 0.001);
        assertEquals(4, qc.w, 0.001);
    }

    @Test
    public void testClone() {
        Quaternion q = new Quaternion(1, 2, 3, 4);
        Quaternion qc = q.clone();
        q.set(4, 5, 6, 7);
        assertEquals(1, qc.x, 0.001);
        assertEquals(2, qc.y, 0.001);
        assertEquals(3, qc.z, 0.001);
        assertEquals(4, qc.w, 0.001);
    }
}

