import XRAnchorMeasurement from './xrAnchorMeasurement';

class SparseBundleAdjustment {
  private measurementNoiseCovariance: Matrix6x6 | null = null; // Measurement noise covariance matrix

  constructor() {}

  /**
   * Calculates the measurement Jacobian matrix for a given anchor measurement and state vector.
   * The specific motion model and measurement function for your XR Anchor Point system should be implemented here.
   * @param anchorMeasurement - The anchor measurement to calculate the Jacobian for.
   * @param stateVector - The current state vector.
   * @returns The measurement Jacobian matrix.
   */
  public calculateMeasurementJacobian(anchorMeasurement: XRAnchorMeasurement, stateVector: Vector6): Matrix6x6 {
    const jacobian = new Matrix6x6();

    jacobian.set(0, 0, 1); // dX/dx
    jacobian.set(0, 1, 0); // dX/dy
    jacobian.set(0, 2, 0); // dX/dz
    jacobian.set(0, 3, stateVector.get(3)); // dX/dvx
    jacobian.set(0, 4, stateVector.get(4)); // dX/dvy
    jacobian.set(0, 5, stateVector.get(5)); // dX/dvz

    jacobian.set(1, 0, 0); // dY/dx
    jacobian.set(1, 1, 1); // dY/dy
    jacobian.set(1, 2, 0); // dY/dz
    jacobian.set(1, 3, stateVector.get(3)); // dY/dvx
    jacobian.set(1, 4, stateVector.get(4)); // dY/dvy
    jacobian.set(1, 5, stateVector.get(5)); // dY/dvz

    jacobian.set(2, 0, 0); // dZ/dx
    jacobian.set(2, 1, 0); // dZ/dy
    jacobian.set(2, 2, 1); // dZ/dz
    jacobian.set(2, 3, stateVector.get(3)); // dZ/dvx
    jacobian.set(2, 4, stateVector.get(4)); // dZ/dvy
    jacobian.set(2, 5, stateVector.get(5)); // dZ/dvz

    return jacobian;
  }

  /**
   * Calculates the measurement Jacobian matrix and residual error vector for bundle adjustment.
   * @param stateVector - The current state vector.
   * @param anchorMeasurements - The list of anchor measurements to use in bundle adjustment.
   * @returns An object containing the measurement Jacobian matrix and residual error vector.
   */
  private calculateJacobianAndResidual(
    stateVector: Vector6,
    anchorMeasurements: XRAnchorMeasurement[]
  ): { jacobian: Matrix6x6; residualError: Vector6 } {
    const numMeasurements = anchorMeasurements.length;
    const jacobian = new Matrix6x6();
    const residualError = new Vector6();

    for (let i = 0; i < numMeasurements; i++) {
      const anchor = anchorMeasurements[i];
      const estimatedMeasurement = this.predictMeasurement(stateVector, anchor);

      // Calculate the residual error vector for this measurement
      const errorVector = this.calculateErrorVector(anchor, estimatedMeasurement);

      // Calculate the Jacobian matrix for this measurement
      const measurementJacobian = this.calculateMeasurementJacobian(anchor, stateVector);
      jacobian.add(measurementJacobian.transpose().multiplyVector(errorVector));

      // Accumulate the residual error
      residualError.add(errorVector);
    }

    return { jacobian, residualError };
  }

  public setMeasurementNoiseCovariance(matrix: Matrix6x6): void {
    this.measurementNoiseCovariance = matrix;
  }

  /**
   * Performs sparse bundle adjustment to update the state vector and covariance matrix.
   * @param stateVector - The current state vector.
   * @param stateCovariance - The current state covariance matrix.
   * @param anchorMeasurements - The list of anchor measurements to use in bundle adjustment.
   * @returns An object containing the updated state vector and covariance matrix.
   */
  public sparseBundleAdjustment(
    stateVector: Vector6,
    stateCovariance: Matrix6x6,
    anchorMeasurements: XRAnchorMeasurement[]
  ): { updatedStateVector: Vector6; updatedCovariance: Matrix6x6 } {
    // Implementation of the sparse bundle adjustment using the Levenberg-Marquardt algorithm
    if (!this.measurementNoiseCovariance) {
      throw new Error('Measurement noise covariance matrix is not defined. Call setMeasurementNoiseCovariance first.');
    }

    const maxIterations = 10; // Set a maximum number of iterations for convergence
    const lambdaInit = 0.01; // Initial damping factor (lambda) to control the step size

    let updatedStateVector = stateVector.clone(); // Initialize the updated state vector
    let updatedCovariance = stateCovariance.clone(); // Initialize the updated state covariance matrix

    for (let i = 0; i < maxIterations; i++) {
      // Calculate the measurement Jacobian matrix and residual error vector
      const { jacobian, residualError } = this.calculateJacobianAndResidual(updatedStateVector, anchorMeasurements);

      // Calculate the weighted Jacobian and residual
      const weightedJacobian = this.weightJacobian(jacobian);
      const weightedResidualError = this.weightResidual(residualError);

      // Compute the Gauss-Newton step and the cost function
      const step = this.computeGaussNewtonStep(weightedJacobian, weightedResidualError);
      const cost = this.calculateCostFunction(weightedResidualError);

      // Apply Levenberg-Marquardt damping factor to control step size
      const lambda = this.updateDampingFactor(lambdaInit, step, cost);
      const adjustedStep = this.applyDampingToStep(step, lambda);

      // Update the state vector and covariance matrix
      updatedStateVector = this.updateStateVector(updatedStateVector, adjustedStep);
      updatedCovariance = this.updateCovarianceMatrix(updatedCovariance, weightedJacobian, lambda);

      // Check for convergence or other stopping criteria (e.g., small step size)
      if (this.isConverged(step)) {
        break;
      }
    }

    return { updatedStateVector, updatedCovariance };
  }

  /**
   * Applies the Levenberg-Marquardt damping factor to the Gauss-Newton step.
   * @param gaussNewtonStep - The Gauss-Newton step vector.
   * @param dampingFactor - The Levenberg-Marquardt damping factor (lambda).
   * @returns The damped Gauss-Newton step vector.
   */
  private applyDampingToStep(gaussNewtonStep: Vector6, dampingFactor: number): Vector6 {
    return gaussNewtonStep.scale(dampingFactor);
  }

  /**
   * Calculates the cost function for bundle adjustment.
   * @param residualError - The residual error vector.
   * @returns The cost function.
   */
  private calculateCostFunction(residualError: Vector6): number {
    // Calculate the weighted residual error
    const weightedResidualError = this.weightResidual(residualError);

    // Compute the squared norm of the weighted residual error vector
    const squaredNorm = weightedResidualError.magnitudeSquared();

    return squaredNorm;
  }

  /**
   * Calculates the error vector between the measured and predicted anchor positions.
   * @param anchor - The anchor measurement.
   * @param predictedMeasurement - The predicted XRRigidTransform measurement.
   * @returns The residual error vector.
   */
  private calculateErrorVector(anchor: XRAnchorMeasurement, predictedMeasurement: XRRigidTransform): Vector6 {
    // Calculate the error in position and orientation
    const errorPosition = {
      x: anchor.position.x - predictedMeasurement.position.x,
      y: anchor.position.y - predictedMeasurement.position.y,
      z: anchor.position.z - predictedMeasurement.position.z,
    };

    const errorOrientation = {
      x: anchor.orientation.x - predictedMeasurement.orientation.x,
      y: anchor.orientation.y - predictedMeasurement.orientation.y,
      z: anchor.orientation.z - predictedMeasurement.orientation.z,
    };

    // Create and return the residual error vector
    return new Vector6([
      errorPosition.x,
      errorPosition.y,
      errorPosition.z,
      errorOrientation.x,
      errorOrientation.y,
      errorOrientation.z,
    ]);
  }

  /**
   * Calculates the measurement Jacobian matrix weight.
   * @param jacobian - The measurement Jacobian matrix.
   * @param measurementNoise - The measurement noise covariance matrix.
   * @returns The measurement Jacobian matrix weight.
   */
  private calculateJacobianWeight(jacobian: Matrix6x6, measurementNoise: Matrix6x6): number {
    // We calculate the weight of the Jacobian matrix based on the measurement noise.
    // A common approach is to use the inverse of the measurement noise covariance matrix.
    // This is known as the data weight and helps to handle measurement uncertainty.

    // If the measurement noise covariance is a diagonal matrix with variances on the diagonal,
    // we can take the reciprocal of each variance element to get the corresponding weight.

    const measurementNoiseInv = measurementNoise.inverse();
    const dataWeight = measurementNoiseInv.getDiagonal();

    // Calculate the mean of data weights for all measurements (diagonal elements)
    const meanDataWeight = dataWeight.reduce((sum, weight) => sum + weight, 0) / 6;

    // Normalize data weights to have the mean data weight as the reference
    const normalizedDataWeight = dataWeight.map(weight => weight / meanDataWeight);

    // Calculate the overall weight by taking the average of the normalized data weights
    const jacobianWeight = normalizedDataWeight.reduce((sum, weight) => sum + weight, 0) / 6;

    return jacobianWeight;
  }

  /**
   * Calculates the measurement residual error weight based on the measurement noise covariance matrix.
   * The specific weighting approach for your XR Anchor Point system should be implemented here.
   * In this simplified implementation, we assume the measurement noise covariance matrix is diagonal,
   * and the weights are obtained by taking the reciprocal of each variance element.
   * @param residualError - The residual error vector.
   * @returns The measurement residual error weight vector.
   */
  private calculateResidualWeight(residualError: Vector6): Vector6 {
    // Calculate the weight for each residual error component using the inverse of the measurement noise covariance matrix
    const residualWeight = residualError.multiplyMatrix(this.measurementNoiseCovariance!.inverse());

    return residualWeight;
  }

  /**
   * Calculates the total squared residual for a given state vector and a list of anchor measurements.
   * @param stateVector - The current state vector.
   * @param anchorMeasurements - The list of anchor measurements to use in bundle adjustment.
   * @returns The total squared residual.
   */
  private calculateTotalSquaredResidual(stateVector: Vector6, anchorMeasurements: XRAnchorMeasurement[]): number {
    let totalSquaredResidual = 0;

    for (const anchor of anchorMeasurements) {
      const estimatedMeasurement = this.predictMeasurement(stateVector, anchor);
      const errorVector = this.calculateErrorVector(anchor, estimatedMeasurement);
      totalSquaredResidual += errorVector.magnitudeSquared();
    }

    return totalSquaredResidual;
  }

  /**
   * Computes the Gauss-Newton step for bundle adjustment.
   * @param jacobian - The measurement Jacobian matrix.
   * @param residualError - The residual error vector.
   * @returns The Gauss-Newton step as a Vector6.
   */
  private computeGaussNewtonStep(jacobian: Matrix6x6, residualError: Vector6): Vector6 {
    const jacobianTranspose = jacobian.transpose();

    // Calculate the normal equation matrix: J^T * J
    const normalMatrix = jacobianTranspose.multiply(jacobian);

    // Add the measurement noise to the diagonal of the normal matrix
    const weightedNormalMatrix = normalMatrix.add(this.measurementNoiseCovariance!.inverse());

    // Calculate the right-hand side: J^T * residualError
    const rhsVector = residualError.multiplyMatrix(jacobianTranspose);

    // Solve for the Gauss-Newton step: step = (J^T * J + measurementNoise)^-1 * (J^T * residualError)
    const step = rhsVector.multiplyMatrix(weightedNormalMatrix.inverse());

    return step;
  }

  /**
   * Checks if the Gauss-Newton step has converged (small step size).
   * @param gaussNewtonStep - The Gauss-Newton step vector.
   * @returns A boolean indicating whether convergence has been reached.
   */
  private isConverged(gaussNewtonStep: Vector6): boolean {
    const epsilon = 1e-6; // Define a small threshold for convergence
    return gaussNewtonStep.magnitude() < epsilon;
  }

  /**
   * Predicts the measurement based on the current state vector and the anchor position.
   * @param stateVector - The current state vector.
   * @param anchor - The anchor measurement for which the prediction is made.
   * @returns The predicted XRRigidTransform measurement.
   */
  private predictMeasurement(stateVector: Vector6, anchor: XRAnchorMeasurement): XRRigidTransform {
    // Extract the position and orientation from the state vector
    const position = {
      x: stateVector.get(0),
      y: stateVector.get(1),
      z: stateVector.get(2),
    };
    const orientation = {
      x: stateVector.get(3),
      y: stateVector.get(4),
      z: stateVector.get(5),
      w: 1,
    };

    // Predict the measurement based on the current state vector
    return new XRRigidTransform(position, orientation);
  }

  private updateCovarianceMatrix(covariance: Matrix6x6, weightedJacobian: Matrix6x6, dampingFactor: number): Matrix6x6 {
    // Calculate the Jacobian transpose and its product with itself (J^T * J)
    const jacobianTranspose = weightedJacobian.transpose();
    const jacobianTransposeProduct = jacobianTranspose.multiply(weightedJacobian);

    // Apply the Levenberg-Marquardt damping factor to the covariance matrix update
    const dampingFactorMatrix = jacobianTransposeProduct.scale(dampingFactor);
    const covarianceUpdate = dampingFactorMatrix.add(new Matrix6x6());

    // Calculate the updated covariance matrix
    return covariance.inverse().add(covarianceUpdate).inverse();
  }

  /**
   * Updates the Levenberg-Marquardt damping factor based on the current step and cost.
   * @param lambdaInit - The initial damping factor.
   * @param gaussNewtonStep - The Gauss-Newton step vector.
   * @param cost - The current cost function value.
   * @returns The updated damping factor.
   */
  private updateDampingFactor(lambdaInit: number, gaussNewtonStep: Vector6, cost: number): number {
    // The damping factor determines the amount of damping to be applied to the step vector.
    // If the damping factor is small, the step size will be larger, and vice versa.
    // Smaller damping factors promote convergence but might cause slower progress.
    // Larger damping factors help with stability but might lead to slower convergence.

    // The update of the damping factor depends on the current cost function value and the progress made.
    // If the optimization is making good progress (reducing the cost), we can decrease the damping factor.
    // If the cost increases, it means we are taking large steps and might need to increase the damping factor.

    const dampingFactorMultiplier = 10.0; // Multiplier for updating the damping factor

    // Check if the step is making progress (reducing the cost)
    const updatedCost = this.calculateCostFunction(gaussNewtonStep);
    const costImprovement = cost - updatedCost;

    // Update the damping factor based on the cost improvement and the damping factor multiplier
    const updatedLambda =
      costImprovement > 0 ? lambdaInit / dampingFactorMultiplier : lambdaInit * dampingFactorMultiplier;

    // Ensure the damping factor stays within a reasonable range
    const minLambda = 1e-9;
    const maxLambda = 1e9;
    const clampedLambda = Math.max(Math.min(updatedLambda, maxLambda), minLambda);

    return clampedLambda;
  }

  /**
   * Updates the state vector and covariance matrix using the Gauss-Newton step.
   * @param stateVector - The current state vector.
   * @param gaussNewtonStep - The Gauss-Newton step vector.
   * @returns The updated state vector.
   */
  private updateStateVector(stateVector: Vector6, gaussNewtonStep: Vector6): Vector6 {
    return stateVector.add(gaussNewtonStep);
  }

  /**
   * Weights the measurement Jacobian matrix to handle measurement noise.
   * @param jacobian - The measurement Jacobian matrix.
   * @param measurementNoise - The measurement noise covariance matrix.
   * @returns The weighted measurement Jacobian matrix.
   */
  private weightJacobian(jacobian: Matrix6x6): Matrix6x6 {
    // Calculate the measurement Jacobian matrix weight
    const jacobianWeight = this.calculateJacobianWeight(jacobian, this.measurementNoiseCovariance!);

    // Apply the weight to the measurement Jacobian matrix
    const weightedJacobian = jacobian.scale(jacobianWeight);

    return weightedJacobian;
  }

  /**
   * Applies weight to the residual error vector to handle measurement uncertainty.
   * @param residualError - The residual error vector.
   * @returns The weighted residual error vector.
   */
  private weightResidual(residualError: Vector6): Vector6 {
    const weight = this.calculateResidualWeight(residualError);
    return residualError.multiply(weight);
  }
}

export default SparseBundleAdjustment;
