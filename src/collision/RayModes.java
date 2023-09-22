package collision;

public enum RayModes {
    CLOSEST(1),
    ANY(2),
    ALL(4);

    private final int value;

    private RayModes(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
