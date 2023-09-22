package shapes;

public class HeightfieldOptions {

    protected double maxValue;
    protected double minValue;
    protected double elementSize;

    /**
     * 
     * @param maxValue 
     * @param minValue
     * @param elementSize 
     */
    public HeightfieldOptions(double maxValue, double minValue, double elementSize){
        this.maxValue = maxValue;
        this.minValue = minValue;
        this.elementSize = elementSize;

    }

    public double getMaxValue() {
        return maxValue;
    }

    public void setMaxValue(double maxValue) {
        this.maxValue = maxValue;
    }

    public double getMinValue() {
        return minValue;
    }

    public void setMinValue(double minValue) {
        this.minValue = minValue;
    }

    public double getElementSize() {
        return elementSize;
    }

    public void setElementSize(double elementSize) {
        this.elementSize = elementSize;
    }
}
