package shapes;

import math.Vec3;
import math.Transform;

import java.util.ArrayList;
import java.util.List;

import collision.AABB;
import utils.Octree;
import math.Quaternion;

/**
 * Trimesh.
 * 
 * @example
 *          // How to make a mesh with a single triangle
 *          const vertices = [
 *          0, 0, 0, // vertex 0
 *          1, 0, 0, // vertex 1
 *          0, 1, 0 // vertex 2
 *          ]
 *          const indices = [
 *          0, 1, 2 // triangle 0
 *          ]
 *          const trimeshShape = new CANNON.Trimesh(vertices, indices)
 */
public class Trimesh extends Shape {

    // changed float to double
    public double[] vertices;
    /**
     * Array of integers, indicating which vertices each triangle consists of. The
     * length of this array is thus 3 times the number of triangles.
     */
    public int[] indices;
    /**
     * The normals data.
     */
    public double[] normals;
    /**
     * The local AABB of the mesh.
     */
    private AABB aabb;
    /**
     * References to vertex pairs, making up all unique edges in the trimesh.
     */
    private int[] edges;
    /**
     * Local scaling of the mesh. Use .setScale() to set it.
     */
    public Vec3 scale;
    /**
     * The indexed triangles. Use .updateTree() to update it.
     */
    public Octree tree;

    public Trimesh(double[] vertices, int[] indices) {
        super(ShapeTypes.TRIMESH);

        this.vertices = vertices;
        this.indices = indices;
        this.normals = new double[indices.length];
        this.aabb = new AABB();
        this.edges = null;
        this.scale = new Vec3(1, 1, 1);
        this.tree = new Octree();

        updateEdges();
        updateNormals();
        updateAABB();
        updateBoundingSphereRadius();
        updateTree();
    }

    // Method to update the Octree structure
    public void updateTree() {
        Octree tree = this.tree;

        tree.reset();
        // changed getAABB() to aabb
        tree.aabb.copy(this.aabb);
        Vec3 scale = this.scale;
        tree.aabb.getLowerBound().x *= 1 / scale.x;
        tree.aabb.getLowerBound().y *= 1 / scale.y;
        tree.aabb.getLowerBound().z *= 1 / scale.z;
        tree.aabb.getUpperBound().x *= 1 / scale.x;
        tree.aabb.getUpperBound().y *= 1 / scale.y;
        tree.aabb.getUpperBound().z *= 1 / scale.z;

        // Insert all triangles
        AABB triangleAABB = new AABB();
        Vec3 a = new Vec3();
        Vec3 b = new Vec3();
        Vec3 c = new Vec3();

        // changed Vec3[] points = { a, b, c }; to
        List<Vec3> points = new ArrayList<Vec3>();
        points.add(a);
        points.add(b);
        points.add(c);
        for (int i = 0; i < this.indices.length / 3; i++) {
            // this.getTriangleVertices(i, a, b, c);

            // Get unscaled triangle verts
            int i3 = i * 3;
            _getUnscaledVertex(this.indices[i3], a);
            _getUnscaledVertex(this.indices[i3 + 1], b);
            _getUnscaledVertex(this.indices[i3 + 2], c);

            triangleAABB.setFromPoints(points);
            // added level : 0
            tree.insert(triangleAABB, i, 0);
        }
        tree.removeEmptyNodes();
    }

    // Method to get triangles within an AABB
     /**
   * Get triangles in a local AABB from the trimesh.
   * @param result An array of integers, referencing the queried triangles.
   */
    // changed int[] to List<Integer> and added parameter result
    public List<Integer> getTrianglesInAABB(AABB aabb, List<Integer> result) {
        AABB unscaledAABB = aabb;
        ; // changed //new AABB(aabb);
        Vec3 scale = this.scale;
        Vec3 lowerBound = unscaledAABB.getLowerBound();
        Vec3 upperBound = unscaledAABB.getUpperBound();
        lowerBound.x /= scale.x;
        lowerBound.y /= scale.y;
        lowerBound.z /= scale.z;
        upperBound.x /= scale.x;
        upperBound.y /= scale.y;
        upperBound.z /= scale.z;

        return tree.aabbQuery(unscaledAABB, result);
    }

    // Method to set the scaling factor
    public void setScale(Vec3 scale) {
        if (this.scale.x != this.scale.y || this.scale.y != this.scale.z) {
            updateNormals();
        }
        this.scale.copy(scale);
        updateAABB();
        updateBoundingSphereRadius();
    }

    // Method to calculate local inertia
    public Vec3 calculateLocalInertia(double mass, Vec3 target) {
        // Approximate with box inertia
        // Exact inertia calculation is overkill
        computeLocalAABB(cli_aabb);
        double x = cli_aabb.getUpperBound().x - cli_aabb.getLowerBound().x;
        double y = cli_aabb.getUpperBound().y - cli_aabb.getLowerBound().y;
        double z = cli_aabb.getUpperBound().z - cli_aabb.getLowerBound().z;

        target.set(
                (1.0f / 12.0f) * mass * (2 * y * 2 * y + 2 * z * 2 * z),
                (1.0f / 12.0f) * mass * (2 * x * 2 * x + 2 * z * 2 * z),
                (1.0f / 12.0f) * mass * (2 * y * 2 * y + 2 * x * 2 * x));

        return target;
    }

    // Method to compute the local AABB of the trimesh
    public void computeLocalAABB(AABB aabb) {
        Vec3 l = aabb.getLowerBound();
        Vec3 u = aabb.getUpperBound();
        int n = this.vertices.length;
        double[] vertices = this.vertices;
        Vec3 v = computeLocalAABB_worldVert;

        getVertex(0, v);
        l.copy(v);
        u.copy(v);

        for (int i = 0; i < n / 3; i++) {
            getVertex(i, v);

            if (v.x < l.x) {
                l.x = v.x;
            } else if (v.x > u.x) {
                u.x = v.x;
            }

            if (v.y < l.y) {
                l.y = v.y;
            } else if (v.y > u.y) {
                u.y = v.y;
            }

            if (v.z < l.z) {
                l.z = v.z;
            } else if (v.z > u.z) {
                u.z = v.z;
            }
        }
    }

    // Method to update the AABB
    public void updateAABB() {
        computeLocalAABB(this.aabb);
    }

    // Method to calculate the bounding sphere radius
    public void updateBoundingSphereRadius() {
        double max2 = 0;
        Vec3 v = new Vec3();
        for (int i = 0, N = this.vertices.length / 3; i < N; i++) {
            getVertex(i, v);
            // changed float to double norm2 and max2
            double norm2 = v.lengthSquared();
            if (norm2 > max2) {
                max2 = norm2;
            }
        }
        this.boundingSphereRadius = Math.sqrt(max2);
    }

    // Method to calculate world AABB
    public void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
        Transform frame = calculateWorldAABB_frame;
        AABB result = calculateWorldAABB_aabb;
        frame.position = pos;
        // changed frame.rotation to frame.quaternion
        frame.quaternion = quat;
        this.aabb.toWorldFrame(frame, result);
        min.copy(result.getLowerBound());
        max.copy(result.getUpperBound());
    }

    // Method to calculate the volume
    // changed return type float to double
    public double volume() {
        return (4.0f * (float) Math.PI * this.boundingSphereRadius) / 3.0f;
    }

    // ... (other imports and class declaration)

    /**
     * Compute the normals of the faces. Will save in the `.normals` array.
     */
    public void updateNormals() {
        Vec3 n = computeNormals_n;

        // Generate normals
        double[] normals = this.normals;
        for (int i = 0; i < this.indices.length / 3; i++) {
            int i3 = i * 3;

            int a = this.indices[i3];
            int b = this.indices[i3 + 1];
            int c = this.indices[i3 + 2];

            getVertex(a, va);
            getVertex(b, vb);
            getVertex(c, vc);

            Trimesh.computeNormal(vb, va, vc, n);

            normals[i3] = n.x;
            normals[i3 + 1] = n.y;
            normals[i3 + 2] = n.z;
        }
    }

    /**
     * Update the `.edges` property
     */
    public void updateEdges() {
        java.util.HashMap<String, Boolean> edges = new java.util.HashMap<>();
        java.util.function.Consumer<Integer> add = (a, b) -> {
            String key = a < b ? a + "_" + b : b + "_" + a;
            edges.put(key, true);
        };
        for (int i = 0; i < this.indices.length / 3; i++) {
            int i3 = i * 3;
            int a = this.indices[i3];
            int b = this.indices[i3 + 1];
            int c = this.indices[i3 + 2];
            add.accept(a, b);
            add.accept(b, c);
            add.accept(c, a);
        }
        java.util.Set<String> keys = edges.keySet();
        this.edges = new int[keys.size() * 2];
        int edgeIndex = 0;
        for (String key : keys) {
            String[] indices = key.split("_");
            this.edges[edgeIndex++] = Short.parseShort(indices[0]);
            this.edges[edgeIndex++] = Short.parseShort(indices[1]);
        }
    }

    /**
     * Get an edge vertex
     * 
     * @param firstOrSecond 0 or 1, depending on which one of the vertices you need.
     * @param vertexStore   Where to store the result
     */
    public void getEdgeVertex(int edgeIndex, int firstOrSecond, Vec3 vertexStore) {
        int vertexIndex = this.edges[edgeIndex * 2 + (firstOrSecond == 1 ? 1 : 0)];
        getVertex(vertexIndex, vertexStore);
    }

    /**
     * Get a vector along an edge.
     */
    public void getEdgeVector(int edgeIndex, Vec3 vectorStore) {
        Vec3 va = getEdgeVector_va;
        Vec3 vb = getEdgeVector_vb;
        getEdgeVertex(edgeIndex, 0, va);
        getEdgeVertex(edgeIndex, 1, vb);
        vb.vsub(va, vectorStore);
    }

    /**
     * Get face normal given 3 vertices
     */
    public static void computeNormal(Vec3 va, Vec3 vb, Vec3 vc, Vec3 target) {
        vb.vsub(va, ab);
        vc.vsub(vb, cb);
        cb.cross(ab, target);
        if (!target.isZero()) {
            target.normalize();
        }
    }

    /**
     * Get vertex i.
     * 
     * @return The "out" vector object
     */
    public Vec3 getVertex(int i, Vec3 out) {
        Vec3 scale = this.scale;
        _getUnscaledVertex(i, out);
        out.x *= scale.x;
        out.y *= scale.y;
        out.z *= scale.z;
        return out;
    }

    /**
     * Get raw vertex i
     * 
     * @return The "out" vector object
     */
    private Vec3 _getUnscaledVertex(int i, Vec3 out) {
        int i3 = i * 3;
        // changed float to double
        double[] vertices = this.vertices;
        // changed out
        return out = new Vec3(vertices[i3], vertices[i3 + 1], vertices[i3 + 2]);
    }

    /**
     * Get a vertex from the trimesh, transformed by the given position and
     * quaternion.
     * 
     * @return The "out" vector object
     */
    public Vec3 getWorldVertex(int i, Vec3 pos, Quaternion quat, Vec3 out) {
        getVertex(i, out);
        Transform.pointToWorldFrame(pos, quat, out, out);
        return out;
    }

    /**
     * Get the three vertices for triangle i.
     */
    public void getTriangleVertices(int i, Vec3 a, Vec3 b, Vec3 c) {
        int i3 = i * 3;
        getVertex(this.indices[i3], a);
        getVertex(this.indices[i3 + 1], b);
        getVertex(this.indices[i3 + 2], c);
    }

    /**
     * Compute the normal of triangle i.
     * 
     * @return The "target" vector object
     */
    public Vec3 getNormal(int i, Vec3 target) {
        int i3 = i * 3;
        // changed target
        return target = new Vec3(this.normals[i3], this.normals[i3 + 1], this.normals[i3 + 2]);
    }

    public static Trimesh createTorus(float radius, float tube, int radialSegments, int tubularSegments, float arc) {
        List<Float> vertices = new ArrayList<>();
        List<Integer> indices = new ArrayList<>();

        for (int j = 0; j <= radialSegments; j++) {
            for (int i = 0; i <= tubularSegments; i++) {
                float u = (i / (float) tubularSegments) * arc;
                float v = (j / (float) radialSegments) * (float) Math.PI * 2;

                float x = (radius + tube * (float) Math.cos(v)) * (float) Math.cos(u);
                float y = (radius + tube * (float) Math.cos(v)) * (float) Math.sin(u);
                float z = tube * (float) Math.sin(v);

                vertices.add(x);
                vertices.add(y);
                vertices.add(z);
            }
        }

        for (int j = 1; j <= radialSegments; j++) {
            for (int i = 1; i <= tubularSegments; i++) {
                int a = (tubularSegments + 1) * j + i - 1;
                int b = (tubularSegments + 1) * (j - 1) + i - 1;
                int c = (tubularSegments + 1) * (j - 1) + i;
                int d = (tubularSegments + 1) * j + i;

                indices.add(a);
                indices.add(b);
                indices.add(d);
                indices.add(b);
                indices.add(c);
                indices.add(d);
            }
        }

        double[] verticesArray = new double[vertices.size()];
        int[] indicesArray = indices.stream().mapToInt(Integer::intValue).toArray();

        for (int i = 0; i < vertices.size(); i++) {
            verticesArray[i] = vertices.get(i);
        }

        return new Trimesh(verticesArray, indicesArray);
    }

    Vec3 computeNormals_n = new Vec3();
    AABB unscaledAABB = new AABB();
    Vec3 getEdgeVector_va = new Vec3();
    Vec3 getEdgeVector_vb = new Vec3();
    // added static
    static Vec3 cb = new Vec3();
    static Vec3 ab = new Vec3();
    static Vec3 va = new Vec3();
    Vec3 vb = new Vec3();
    Vec3 vc = new Vec3();
    AABB cli_aabb = new AABB();
    Vec3 computeLocalAABB_worldVert = new Vec3();
    Transform calculateWorldAABB_frame = new Transform();
    AABB calculateWorldAABB_aabb = new AABB();

    // Define other private helper methods...
}
