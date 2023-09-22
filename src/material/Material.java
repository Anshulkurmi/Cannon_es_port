package material;

/**
 * Defines a physics material.
 */
public class Material {
    /**
   * Material name.
   * If options is a string, name will be set to that string.
   * @todo Deprecate this
   */
    public String name;
    /** Material id. */
    public int id;
    /**
     * Friction for this material.
     * If non-negative, it will be used instead of the friction given by ContactMaterials.
     * If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
     */
    public double friction;
    /**
     * Restitution for this material.
     * If non-negative, it will be used instead of the restitution given by ContactMaterials.
     * If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
     */
    public double restitution;

    /** Counter for generating unique material IDs. */
    public static int idCounter = 0;

    /**
     * Default constructor.
     * Initializes a Material with empty name, -1 friction, and -1 restitution.
     */
    public Material() {
        this("", -1, -1);
    }

    /**
     * Constructor with a material name.
     * Initializes a Material with the given name, -1 friction, and -1 restitution.
     * @param name The name of the material.
     */
    public Material(String name) {
        this(name, -1, -1);
    }

    
    /**
    * Restitution for this material.
    * If non-negative, it will be used instead of the restitution given by ContactMaterials. If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
    */
    /**
     * 
     * Constructor with friction and restitution values.
     * Initializes a Material with an empty name, the given friction, and the given restitution.
     * @param friction The friction coefficient for the material.
    * Friction for this material.
    * If non-negative, it will be used instead of the friction given by ContactMaterials. If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
    
     * @param restitution The restitution (bounciness) for the material.
     * Restitution for this material.
    * If non-negative, it will be used instead of the restitution given by ContactMaterials. If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
     */
    public Material(double friction, double restitution) {
        this("", friction, restitution);
    }

    /**
     * Constructor with a material name, friction, and restitution values.
     * Initializes a Material with the given name, friction, and restitution.
     * @param name The name of the material.
     * @param friction The friction coefficient for the material.
     * @param restitution The restitution (bounciness) for the material.
     */
    public Material(String name, double friction, double restitution) {
        this.name = name;
        this.id = idCounter++;
        // Ensure that friction and restitution are non-negative or set them to -1 otherwise.
        this.friction = friction >= 0 ? friction : -1;
        this.restitution = restitution >= 0 ? restitution : -1;
    }
}

