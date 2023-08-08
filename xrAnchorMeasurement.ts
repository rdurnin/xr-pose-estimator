import { Matrix, Quaternion, Vector3 } from '@babylonjs/core';

class XRAnchorMeasurement {
  anchorId: string;
  timestamp: number;
  poseMatrix: Matrix;

  constructor(anchorId: string, timestamp: number, poseMatrix: Matrix) {
    this.anchorId = anchorId;
    this.timestamp = timestamp;
    this.poseMatrix = poseMatrix;
  }

  /**
   * Extracts the position (translation) from the pose matrix.
   * @returns The position vector as a Vector3.
   */
  public get position(): Vector3 {
    return this.poseMatrix.getTranslation();
  }

  /**
   * Extracts the orientation (rotation) from the pose matrix.
   * @returns The orientation quaternion as a Quaternion.
   */
  public get orientation(): Quaternion {
    return Quaternion.FromRotationMatrix(this.poseMatrix);
  }
}

export default XRAnchorMeasurement;
