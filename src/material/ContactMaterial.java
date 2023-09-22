package material;

/**
 * Defines what happens when two materials meet.
 * @todo Refactor materials to materialA and materialB
 */
public class ContactMaterial {
    /**
     * Identifier of this material.
     */
    public int id;
    /**
     * Participating materials.
     */
    public Material[] materials;
    /**
     * Friction coefficient.
     * @default 0.3
     */
    public double friction;
    /**
     * Restitution coefficient.
     * @default 0.3
     */
    public double restitution;
    /**
     * Stiffness of the produced contact equations.
     * @default 1e7
     */
    public double contactEquationStiffness;
    /**
     * Relaxation time of the produced contact equations.
     * @default 3
     */
    public double contactEquationRelaxation;
    /**
     * Stiffness of the produced friction equations.
     * @default 1e7
     */
    public double frictionEquationStiffness;
    /**
     * Relaxation time of the produced friction equations.
     * @default 3
     */
    public double frictionEquationRelaxation;

    /** Counter for generating unique material IDs. */
    public static int idCounter = 0;

    /**
     * Default constructor.
     * Initializes a ContactMaterial with default values.
     * @param m1 The first material.
     * @param m2 The second material.
     */
    public ContactMaterial(Material m1, Material m2) {
        this(m1, m2, /*0.3, 0.3, 1e7, 3, 1e7, 3*/ new ContactMaterialOptions());
    }

    /**
     * Constructor with custom parameters.
     * Initializes a ContactMaterial with the specified parameters.
     * @param m1 The first material.
     * @param m2 The second material.
     * @param friction Friction coefficient.
     * @param restitution Restitution coefficient.
     * @param contactEquationStiffness Stiffness of contact equations.
     * @param contactEquationRelaxation Relaxation time of contact equations.
     * @param frictionEquationStiffness Stiffness of friction equations.
     * @param frictionEquationRelaxation Relaxation time of friction equations.
     */
    public ContactMaterial(
        Material m1,
        Material m2,
        /*double friction,
        double restitution,
        double contactEquationStiffness,
        double contactEquationRelaxation,
        double frictionEquationStiffness,
        double frictionEquationRelaxation */
        ContactMaterialOptions options
    ) {
        this.id = ContactMaterial.idCounter++;
        this.materials = new Material[] { m1, m2 };
        this.friction = options.friction;
        this.restitution = options.restitution;
        this.contactEquationStiffness = options.contactEquationStiffness;
        this.contactEquationRelaxation = options.contactEquationRelaxation;
        this.frictionEquationStiffness = options.frictionEquationStiffness;
        this.frictionEquationRelaxation = options.frictionEquationRelaxation;
    }
}
