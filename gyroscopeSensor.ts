import { Quaternion, Vector3 } from '@babylonjs/core';
import { Vector3Filter } from 'api/babylon/scripts/filter';

class GyroscopeSensor {
  private sensor: Gyroscope | null;
  private isAvailable: boolean;
  private calibrationOffset: Vector3;

  private lastTimestamp: number;
  private lastOrientation: Quaternion;

  private readonly frequency = 60;
  private readonly oneEuroFilter = new Vector3Filter(this.frequency, 0.685, 0.1, 1.0);

  constructor() {
    this.sensor = null;
    this.isAvailable = 'Gyroscope' in window;
    this.calibrationOffset = new Vector3(0, 0, 0);

    this.lastTimestamp = 0;
    this.lastOrientation = new Quaternion();
  }

  public isSensorAvailable(): boolean {
    return this.isAvailable;
  }

  public async configureSensor(): Promise<void> {
    if (!this.isAvailable) {
      throw new Error('Gyroscope sensor is not available on this device.');
    }

    try {
      this.sensor = await new Gyroscope({ frequency: this.frequency });
    } catch (error) {
      throw new Error('Error configuring gyroscope sensor: ' + error.message);
    }
  }

  /**
   * Gets the measurement noise covariance matrix for the gyroscope sensor.
   * @param deltaTime - The time elapsed since the last sensor reading in seconds.
   * @returns The measurement noise covariance matrix (Matrix6x6).
   */
  public getMeasurementNoiseCovariance(): Matrix6x6 {
    const currentTimestamp = Date.now();
    const deltaTime = (currentTimestamp - this.lastTimestamp) / 1000; // Convert to seconds

    const currentEuler = this.readSensorData();
    const currentOrientation = Quaternion.FromEulerAngles(currentEuler.x, currentEuler.y, currentEuler.z);

    // Calculate the change in orientation since the last sensor reading
    const deltaOrientation = this.calculateDeltaOrientation(currentOrientation, this.lastOrientation);

    // Calculate the angular velocity using the change in orientation and time elapsed
    const angularVelocity = this.calculateAngularVelocity(deltaOrientation, deltaTime);

    // Assume that the standard deviation of each angular velocity component is equal
    const sigmaAngularVelocity = angularVelocity.map(angularVelocity => {
      // Calculate the standard deviation from the noise characteristics of the sensor
      // For example, using the sensor noise specifications, calibration data, or other methods.
      // Replace the value below with the correct calculation based on your sensor characteristics.
      return 0.001; // Example value, adjust based on your sensor noise characteristics
    });

    // Construct the measurement noise covariance matrix
    const measurementNoiseCovariance = new Matrix6x6([
      sigmaAngularVelocity[0] * sigmaAngularVelocity[0],
      0,
      0,
      0,
      0,
      0,
      0,
      sigmaAngularVelocity[1] * sigmaAngularVelocity[1],
      0,
      0,
      0,
      0,
      0,
      0,
      sigmaAngularVelocity[2] * sigmaAngularVelocity[2],
      0,
      0,
      0,
      0,
      0,
      0,
      sigmaAngularVelocity[0] * sigmaAngularVelocity[0],
      0,
      0,
      0,
      0,
      0,
      0,
      sigmaAngularVelocity[1] * sigmaAngularVelocity[1],
      0,
      0,
      0,
      0,
      0,
      0,
      sigmaAngularVelocity[2] * sigmaAngularVelocity[2],
    ]);

    // Store the current timestamp and orientation for the next sensor reading
    this.lastOrientation = currentOrientation;
    this.lastTimestamp = currentTimestamp;

    return measurementNoiseCovariance;
  }

  public readSensorData(): Vector3 {
    if (!this.sensor) {
      return Vector3.Zero();
    }

    return this.oneEuroFilter
      .filter(new Vector3(this.sensor.x || 0, this.sensor.y || 0, this.sensor.z || 0), this.sensor.timestamp)
      .subtract(this.calibrationOffset);
  }

  public setCalibrationOffset(offset: Vector3): void {
    this.calibrationOffset = offset;
  }
  // Add interactive calibration methods for a functional web application here...

  /**
   * Calculates the change in orientation between two XRQuaternion objects.
   * @param currentOrientation - The current orientation.
   * @param lastOrientation - The orientation from the last sensor reading.
   * @returns The change in orientation as a XRQuaternion.
   */
  private calculateDeltaOrientation(currentOrientation: Quaternion, lastOrientation: Quaternion): Quaternion {
    // The change in orientation is calculated as the relative rotation from the last orientation
    // to the current orientation.
    const deltaOrientation = currentOrientation.clone().multiply(Quaternion.Inverse(lastOrientation));
    return deltaOrientation;
  }

  /**
   * Calculates the angular velocity given the change in orientation and time elapsed.
   * @param deltaOrientation - The change in orientation as a Quaternion.
   * @param deltaTime - The time elapsed since the last sensor reading in seconds.
   * @returns The angular velocity as an array of three components [x, y, z].
   */
  private calculateAngularVelocity(deltaOrientation: Quaternion, deltaTime: number): number[] {
    // Extract the axis of rotation from the delta orientation
    const eulerAngles = deltaOrientation.toEulerAngles();

    const axis = new Vector3(eulerAngles.x, eulerAngles.y, eulerAngles.z).normalize();
    const angle = eulerAngles.length();

    // Calculate the angular velocity vector by dividing the angle by the time elapsed
    const angularVelocity = axis.scale(angle / deltaTime);

    return [angularVelocity.x, angularVelocity.y, angularVelocity.z];
  }
}

export default GyroscopeSensor;
