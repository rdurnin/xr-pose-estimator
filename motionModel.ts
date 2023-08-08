import { Vector3 } from '@babylonjs/core';

/**
 * Represents a motion model for predicting the state of XR anchors based on constant velocity.
 * This model assumes that the XR anchor is moving with a constant velocity in 3D space.
 */
class MotionModel {
  /**
   * Predicts the new state and calculates the motion Jacobian based on the previous state and elapsed time.
   * @param stateVector - The current state vector containing position and orientation information.
   * @param gyroscopeData - The gyroscope sensor data containing angular velocity.
   * @param deltaTime - The time elapsed since the last prediction in seconds.
   * @returns An object containing the predicted state and the motion Jacobian matrix.
   */
  public predict(
    stateVector: Vector6,
    gyroscopeData: Vector3,
    deltaTime: number
  ): { predictedState: Vector6; motionJacobian: Matrix6x6 } {
    // Implement the motion model to predict the new state based on the previous state and elapsed time
    const predictedState = this.calculatePredictedState(stateVector, gyroscopeData, deltaTime);

    // Calculate the Jacobian matrix for the motion model (in this case, an identity matrix)
    const motionJacobian: Matrix6x6 = this.calculateMotionJacobian(stateVector, deltaTime);

    return { predictedState, motionJacobian };
  }

  /**
   * Calculates the motion vector representing the change in position and orientation based on constant velocity.
   * @param stateVector - The current state vector containing position and orientation information.
   * @param gyroscopeData - The gyroscope sensor data containing angular velocity.
   * @param deltaTime - The time elapsed since the last prediction in seconds.
   * @returns The motion vector as a Vector6 containing position and angular velocity information.
   */
  public predictMotionVector(stateVector: Vector6, gyroscopeData: Vector3, deltaTime: number): Vector6 {
    // Constants for constant velocity motion model
    const velocityMagnitude = 1.0; // Replace with the actual velocity magnitude based on the application

    // Extract current position and orientation from the state vector
    const [x, y, z, roll, pitch, yaw] = stateVector.toArray();

    // Calculate the motion vector based on the constant velocity model
    const cosYaw = Math.cos(yaw);
    const cosPitch = Math.cos(pitch);
    const sinYaw = Math.sin(yaw);
    const sinPitch = Math.sin(pitch);

    const dx = velocityMagnitude * cosYaw * cosPitch * deltaTime;
    const dy = velocityMagnitude * sinYaw * cosPitch * deltaTime;
    const dz = velocityMagnitude * sinPitch * deltaTime;

    // Use gyroscope sensor data to predict angular velocity (roll, pitch, yaw rates)
    const angularVelocity = gyroscopeData.scale(deltaTime);

    // Create the motion vector representing the change in position and orientation
    const motionVector = new Vector6([dx, dy, dz, angularVelocity.x, angularVelocity.y, angularVelocity.z]);

    return motionVector;
  }

  /**
   * Calculates the motion Jacobian matrix used for state covariance propagation in the XRAnchorPoseEstimator class.
   * For the constant velocity motion model, the Jacobian matrix is an identity matrix.
   * @param stateVector - The current state vector containing position and orientation information.
   * @param deltaTime - The time elapsed since the last prediction in seconds.
   * @returns The motion Jacobian matrix as a Matrix6x6.
   */
  private calculateMotionJacobian(stateVector: Vector6, deltaTime: number): Matrix6x6 {
    // For the constant velocity motion model, the Jacobian matrix is an identity matrix.
    return new Matrix6x6();
  }

  /**
   * Calculates the predicted state by combining the current state with the motion vector.
   * @param stateVector - The current state vector containing position and orientation information.
   * @param gyroscopeData - The gyroscope sensor data containing angular velocity.
   * @param deltaTime - The time elapsed since the last prediction in seconds.
   * @returns The predicted state vector as a Vector6 containing updated position and orientation information.
   */
  private calculatePredictedState(stateVector: Vector6, gyroscopeData: Vector3, deltaTime: number): Vector6 {
    // This function combines the current state with the predicted motion vector to calculate the new state vector.
    const motionVector = this.predictMotionVector(stateVector, gyroscopeData, deltaTime);
    stateVector.addInPlace(motionVector);
    return stateVector;
  }
}

export default MotionModel;
