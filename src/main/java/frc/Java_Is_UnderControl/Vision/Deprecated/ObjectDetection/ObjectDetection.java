package frc.Java_Is_UnderControl.Vision.Deprecated.ObjectDetection;

public class ObjectDetection {
  public final double ty;
  public final double tx;
  public final String objectName;

  public ObjectDetection(double tx, double ty, String objectName) {
    this.tx = tx;
    this.ty = ty;
    this.objectName = objectName;
  }
}
