class Matrix6x6 {
  private readonly data: Float32Array;

  constructor(values?: number[]) {
    this.data = new Float32Array(36); // 6x6 matrix
    if (values) {
      this.setValues(values);
    } else {
      this.setToIdentity();
    }
  }

  public get(row: number, column: number): number {
    return this.data[row * 6 + column];
  }

  public getDiagonal(): Vector6 {
    const diagonalData = this.data.map((row, i) => row[i]);
    return new Vector6(diagonalData);
  }

  public set(row: number, column: number, value: number): void {
    this.data[row * 6 + column] = value;
  }

  public setDiagonal(diagonalValues: number[]): Matrix6x6 {
    if (diagonalValues.length !== 6) {
      throw new Error('Diagonal values must be an array of length 6.');
    }

    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        if (i === j) {
          this.elements[i][j] = diagonalValues[i];
        }
      }
    }

    return this;
  }

  public setValues(values: number[]): void {
    if (values.length !== 36) {
      throw new Error('Vector6 constructor requires exactly 6 values.');
    }
    for (let i = 0; i < 36; i++) {
      this.data[i] = values[i];
    }
  }

  public setToIdentity(): void {
    this.data[0] = this.data[7] = this.data[14] = this.data[21] = this.data[28] = this.data[35] = 1;
    for (let i = 0; i < 36; i++) {
      if (i !== 0 && i !== 7 && i !== 14 && i !== 21 && i !== 28 && i !== 35) {
        this.data[i] = 0;
      }
    }
  }

  public toArray(): number[] {
    return Array.from(this.data);
  }

  public add(matrix: Matrix6x6): Matrix6x6 {
    const result: number[] = [];

    for (let i = 0; i < 36; i++) {
      result.push(this.data[i] + matrix.data[i]);
    }

    return new Matrix6x6(result);
  }

  public addInPlace(matrix: Matrix6x6): Matrix6x6 {
    for (let i = 0; i < 36; i++) {
      this.data[i] += matrix.data[i];
    }
    return this;
  }

  public clone(): Matrix6x6 {
    return new Matrix6x6(this.toArray());
  }

  public inverse(): Matrix6x6 {
    const det = this.calculateDeterminant();

    if (Math.abs(det) < Number.EPSILON) {
      throw new Error('Matrix is singular. Cannot find inverse.');
    }

    const adjugate = this.calculateAdjugate();
    const inverseData = adjugate.getData().map(row => row.map(val => val / det));

    return new Matrix6x6(inverseData);
  }

  public multiply(matrix: Matrix6x6): Matrix6x6 {
    const result: number[] = Array(36).fill(0);

    for (let row = 0; row < 6; row++) {
      for (let col = 0; col < 6; col++) {
        for (let k = 0; k < 6; k++) {
          result[row * 6 + col] += this.data[row * 6 + k] * matrix.data[k * 6 + col];
        }
      }
    }

    return new Matrix6x6(result);
  }

  public multiplyVector(vector: Vector6): Matrix6x6 {
    const resultData = Array.from({ length: 6 }, () => Array(6).fill(0));

    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        resultData[i][j] = this.data[i][j] * vector.get(j);
      }
    }

    return new Matrix6x6(resultData);
  }

  public scale(scalar: number): Matrix6x6 {
    const result = new Matrix6x6();
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        result.set(i, j, this.get(i, j) * scalar);
      }
    }
    return result;
  }

  public transpose(): Matrix6x6 {
    const transposedMatrix = new Matrix6x6();
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        transposedMatrix.set(i, j, this.get(j, i));
      }
    }
    return transposedMatrix;
  }

  public calculateDeterminant(): number {
    if (this.data.length !== 36) {
      throw new Error('Matrix must be 6x6 to calculate determinant.');
    }

    const luDecomposition = this.performLuDecomposition();

    const determinant = luDecomposition.u.reduce((acc, row, i) => acc * row[i], 1);
    const parity = luDecomposition.p.reduce((acc, value, i) => acc * (i === value ? 1 : -1), 1);

    return determinant * parity;
  }

  public calculateAdjugate(): Matrix6x6 {
    if (this.data.length !== 36) {
      throw new Error('Matrix must be 6x6 to calculate adjugate.');
    }

    const luDecomposition = this.performLuDecomposition();

    const { l, u, p } = luDecomposition;
    const adjugateData: number[][] = Array.from({ length: 6 }, () => Array(6).fill(0));

    for (let i = 0; i < 6; i++) {
      adjugateData[i][i] = 1;
      for (let j = i + 1; j < 6; j++) {
        for (let k = 0; k < 6; k++) {
          adjugateData[j][k] -= l.get(j, i) * adjugateData[i][k];
        }
      }
    }

    for (let i = 5; i >= 0; i--) {
      for (let k = 0; k < 6; k++) {
        for (let j = i + 1; j < 6; j++) {
          adjugateData[i][k] -= u.get(i, j) * adjugateData[j][k];
        }
        adjugateData[i][k] /= u.get(i, i);
      }
    }

    const adjugateDataPermuted: number[][] = Array.from({ length: 6 }, () => Array(6).fill(0));
    for (let i = 0; i < 6; i++) {
      const row = p.indexOf(i);
      for (let j = 0; j < 6; j++) {
        adjugateDataPermuted[i][j] = adjugateData[row][j];
      }
    }

    return new Matrix6x6(adjugateDataPermuted);
  }

  private performLuDecomposition(): { l: Matrix6x6; u: Matrix6x6; p: number[] } {
    const l = new Matrix6x6();
    const u = new Matrix6x6();
    const p: number[] = Array.from({ length: 6 }, (_, i) => i);

    for (let i = 0; i < 6; i++) {
      let pivotIndex = i;
      let pivotValue = Math.abs(this.data[i * 6 + i]);

      for (let j = i + 1; j < 6; j++) {
        const currentValue = Math.abs(this.data[j * 6 + i]);
        if (currentValue > pivotValue) {
          pivotIndex = j;
          pivotValue = currentValue;
        }
      }

      if (pivotValue === 0) {
        throw new Error('Matrix is singular. Cannot perform LU decomposition.');
      }

      if (pivotIndex !== i) {
        this.swapRows(i, pivotIndex);
        [p[i], p[pivotIndex]] = [p[pivotIndex], p[i]];
      }

      // Calculate the LU matrices
      l.set(i, i, 1);
      for (let j = i + 1; j < 6; j++) {
        const factor = this.data[j * 6 + i] / this.data[i * 6 + i];
        l.set(j, i, factor);
        for (let k = i; k < 6; k++) {
          this.data[j * 6 + k] -= factor * this.data[i * 6 + k];
        }
      }
      for (let j = i; j < 6; j++) {
        u.set(i, j, this.data[i * 6 + j]);
      }
    }

    return { l, u, p };
  }

  private swapRows(row1: number, row2: number): void {
    for (let j = 0; j < 6; j++) {
      const temp = this.data[row1 * 6 + j];
      this.data[row1 * 6 + j] = this.data[row2 * 6 + j];
      this.data[row2 * 6 + j] = temp;
    }
  }
}

class Vector6 {
  private readonly data: Float32Array;

  constructor(values?: number[]) {
    this.data = new Float32Array(6); // 6-dimensional vector
    if (values) {
      this.setValues(values);
    } else {
      this.setZeros();
    }
  }

  public get(index: number): number {
    return this.data[index];
  }

  public set(index: number, value: number): void {
    this.data[index] = value;
  }

  public setValues(values: number[]): void {
    if (values.length !== 6) {
      throw new Error('Vector6 constructor requires exactly 6 values.');
    }
    for (let i = 0; i < 6; i++) {
      this.data[i] = values[i];
    }
  }

  public setZeros(): void {
    for (let i = 0; i < 6; i++) {
      this.data[i] = 0;
    }
  }

  public toArray(): number[] {
    return Array.from(this.data);
  }

  public add(vector: Vector6): Vector6 {
    const result = new Vector6();
    for (let i = 0; i < 6; i++) {
      result.set(i, this.get(i) + vector.get(i));
    }
    return result;
  }

  /**
   * Adds the values of another Vector6 to this vector in-place.
   * @param vector - The vector to add.
   * @returns This vector after the addition.
   */
  public addInPlace(vector: Vector6): Vector6 {
    for (let i = 0; i < 6; i++) {
      this.data[i] += vector.data[i];
    }
    return this;
  }

  public clone(): Vector6 {
    return new Vector6(this.toArray());
  }

  public dot(vector: Vector6): number {
    let dotProduct = 0;
    for (let i = 0; i < 6; i++) {
      dotProduct += this.get(i) * vector.get(i);
    }
    return dotProduct;
  }

  public magnitude(): number {
    return Math.sqrt(this.elements.reduce((sum, element) => sum + element ** 2, 0));
  }

  public magnitudeSquared(): number {
    return this.data.reduce((sum, value) => sum + value * value, 0);
  }

  public map(fn: (value: number, index: number) => number): Vector6 {
    const mappedData = this.data.map(fn);
    return new Vector6(mappedData);
  }

  public multiply(vector: Vector6): Vector6 {
    const result = new Vector6();
    for (let i = 0; i < 6; i++) {
      result.set(i, this.data[i] * vector.get(i));
    }
    return result;
  }

  public multiplyMatrix(matrix: Matrix6x6): Vector6 {
    if (!(matrix instanceof Matrix6x6)) {
      throw new Error('Input must be a Matrix6x6');
    }

    const resultArray: number[] = [];
    for (let i = 0; i < 6; i++) {
      let sum = 0;
      for (let j = 0; j < 6; j++) {
        sum += this.elements[j] * matrix.get(j, i); // Transposed matrix multiplication
      }
      resultArray.push(sum);
    }

    return new Vector6(resultArray);
  }

  public normalize(): Vector6 {
    const magnitude = this.magnitude();
    if (magnitude === 0) {
      throw new Error('Cannot normalize the zero vector.');
    }

    const normalizedElements = this.elements.map(element => element / magnitude);
    return new Vector6(normalizedElements);
  }

  public reduce(fn: (accumulator: number, value: number) => number, initialValue: number): number {
    return this.data.reduce(fn, initialValue);
  }

  public scale(scalar: number): Vector6 {
    const result = new Vector6();
    for (let i = 0; i < 6; i++) {
      result.set(i, this.get(i) * scalar);
    }
    return result;
  }

  public subtract(vector: Vector6): Vector6 {
    const result = new Vector6();
    for (let i = 0; i < 6; i++) {
      result.set(i, this.get(i) - vector.get(i));
    }
    return result;
  }
}
