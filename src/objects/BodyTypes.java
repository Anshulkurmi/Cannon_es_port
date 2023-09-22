package objects;

/**
 * BodyType enum
 */
public enum BodyTypes {
    DYNAMIC(1),
    STATIC(2),
    KINEMATIC(4);

    private final int value;

    private BodyTypes(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
