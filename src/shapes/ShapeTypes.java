package shapes;

/**
 * Enum for shape types.
 */
public  enum ShapeTypes {
	SPHERE(1), PLANE(2), BOX(4), COMPOUND(8), CONVEXPOLYHEDRON(16), HEIGHTFIELD(32), PARTICLE(64), CYLINDER(128),
	TRIMESH(256);

	private final int value;

	ShapeTypes(int value) {
		this.value = value;
	}

	public int getValue() {
		return value;
	}
}