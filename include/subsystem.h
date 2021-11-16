// This class represents the physical system.
// Change all Alice to BOB?
#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H
#include "fixedPoint.h"
#include <math.h>

class subSystem {
public:
  // declares the constants for the system.
  // _ne refers to variables in plaintext
  double **xk_ne;
  double **uk_ne;
  fixedPoint **uk;
  int sizeuk[2];
  int sizexk[2];

  fixedPoint **yk;
  fixedPoint **yd;
  int sizeyk[2];
  int sizeyd[2];

  fixedPoint dFLi;
  fixedPoint dFL;

  fixedPoint **sinCoeff;
  fixedPoint **cosCoeff;
  int sizeTaylor[2];

  

  subSystem() {}

  

  // Simulates one time step of the system given the control action u[k]
  // TODO: change name. This is a simulation of the system evolution, not a measurement
  //       add noise
  void measureState(fixedPoint **uk) {
    for (int i = 0; i < this->sizeuk[0]; i++) {
      for (int j = 0; j < this->sizeuk[1]; j++) {
        // get the control action
        this->uk_ne[i][j] = uk[i][j].reveal<double>(ALICE)*0.4;
      }
    }
    double dt = 0.1;
    double step = 100;
    for ( int i = 0; i < step; i++ ){
      this->xk_ne[0][0] = this->xk_ne[0][0] + this->uk_ne[0][0] * cos( this->xk_ne[2][0] )*dt/step;
      this->xk_ne[1][0] = this->xk_ne[1][0] + this->uk_ne[0][0] * sin( this->xk_ne[2][0] )*dt/step;
      this->xk_ne[2][0] = this->xk_ne[2][0] + this->uk_ne[1][0] * dt/step;
    }

  }

  // Computes the measurements y[k] 
  // Updates yk to use it in the emp library for the next iteration
  void computeyk() {
    for (int i = 0; i < this->sizeyk[0]; i++) {
      for (int j = 0; j < this->sizeyk[1]; j++) {
        this->yk[i][j] = fixedPoint(this->xk_ne[i][j], 24, 24, ALICE);
      }
    }
  } 

  


///////////////////////////////////////////////////////////////////////////
// Functions required for the operations. Not focused on the control system
///////////////////////////////////////////////////////////////////////////

  // Computes the multiplication between matrices A and B
  void matrixMul(fixedPoint **A, fixedPoint **B, fixedPoint **ret, int *ASize,
                 int *BSize) {
    fixedPoint zero(0, 24, 24, PUBLIC);
    for (int i = 0; i < ASize[0]; i++) {
      for (int j = 0; j < BSize[1]; j++) {
        ret[i][j] = zero;
        for (int k = 0; k < ASize[1]; k++) {
          ret[i][j] = ret[i][j] + (A[i][k] * B[k][j]);
        }
      }
    }
  }

  // Computes the multiplication between matrices A and B which are in plaintext
  void matrixMulNE(double **A, double **B, double **ret, int *ASize,
                 int *BSize) {
    for (int i = 0; i < ASize[0]; i++) {
      for (int j = 0; j < BSize[1]; j++) {
        ret[i][j] = 0;
        for (int k = 0; k < ASize[1]; k++) {
          ret[i][j] = ret[i][j] + (A[i][k] * B[k][j]);
        }
      }
    }
  }

  // Computes the multiplication between matrix A and vector B
  void matrixVecMul(fixedPoint **A, fixedPoint **B, fixedPoint *ret,
                    int *size) {
    fixedPoint zero(0, 24, 24, PUBLIC);
    for (int i = 0; i < size[0]; i++) {
      ret[i] = zero;
      for (int j = 0; j < size[1]; j++) {
        ret[i] = ret[i] + ((A[i][j]) * (B[j][0]));
      }
    }
  }
  // Computes the multiplication between matrix A and vector B which are in plaintext
  void matrixVecMulNE(double **A, double **B, double *ret,
                    int *size) {
    for (int i = 0; i < size[0]; i++) {
      ret[i] = 0;
      for (int j = 0; j < size[1]; j++) {
        ret[i] = ret[i] + ((A[i][j]) * (B[j][0]));
      }
    }
  }

  // Reads the size of matrices stored inside the file input
  void getFileSize(string input, int *size) {
    int rowSize = 0, colSize = 0;
    fstream file;
    string inputLine, stringinput;
    // cout<<input<<endl;
    file.open(input, ios::in);
    if (!file.is_open()) {
      cout << "ERROR: file not opened" << endl;
      return;
    }
    while (getline(file, inputLine)) {
      rowSize++;
      stringstream line(inputLine);
      if (rowSize == 1) {
        while (getline(line, stringinput, ',')) {
          colSize++;
        }
      }
    }
    size[0] = rowSize;
    size[1] = colSize;
    file.close();
  }

  // Retrieves the matrices stored inside the file input
  void readFile(double **data, string input, int *size) {
    int i = 0;
    int j = 0;
    fstream file;
    string inputLine, stringinput;
    file.open(input, ios::in);
    while (getline(file, inputLine)) {
      j = 0;
      stringstream line(inputLine);
      while (getline(line, stringinput, ',')) {
        data[i][j] = stod(stringinput);
        // cout << "load data" << stringinput << endl;
        j++;
      }
      i++;
    }
  }

  // TODO: load data sizes and initialize robot state
  void initializeData(){
    this->dFLi = fixedPoint(1/0.035, 24, 24, ALICE);
    this->dFL  = fixedPoint(0.035, 24, 24, ALICE);
    this->sizeuk[0] = 2;
    this->sizeuk[1] = 1;

    getFileSize("Data/x0.txt", this->sizexk);
    getFileSize("Data/x0.txt", this->sizeyk);
    this->xk_ne = new double*[this->sizexk[0]];
    this->yk = new fixedPoint*[this->sizexk[0]];
    double **datax0;
    datax0 = new double*[this->sizexk[0]];
    for(int i = 0; i<this->sizexk[0]; i++){
      this->xk_ne[i] = new double[this->sizexk[1]];
      this->yk[i] = new fixedPoint[this->sizexk[1]];
      datax0[i] = new double[this->sizexk[1]];
    }
    readFile(datax0, "Data/x0.txt", this->sizexk);
    for (int i = 0; i < this->sizexk[0]; i++) {
      for (int j = 0; j < this->sizexk[1]; j++) {
        this->yk[i][j] = fixedPoint(datax0[i][j], 24, 24, ALICE);
        this->xk_ne[i][j] = datax0[i][j];
      }
    }

    this->uk_ne = new double *[this->sizeuk[0]];
    for (int i = 0; i < this->sizeuk[0]; i++) {
      this->uk_ne[i] = new double[this->sizeuk[1]];
    }

    double **dataSinCoeff;
    double **dataCosCoeff;
    getFileSize("Data/sinCoeff.txt", this->sizeTaylor);
    this->sinCoeff = new fixedPoint*[this->sizeTaylor[0]];
    this->cosCoeff = new fixedPoint*[this->sizeTaylor[0]];
    dataSinCoeff = new double*[this->sizeTaylor[0]];
    dataCosCoeff = new double*[this->sizeTaylor[0]];
    for(int i=0; i < this->sizeTaylor[0]; i++){
      this->sinCoeff[i] = new fixedPoint[this->sizeTaylor[1]];
      this->cosCoeff[i] = new fixedPoint[this->sizeTaylor[1]];
      dataSinCoeff[i] = new double[this->sizeTaylor[1]];
      dataCosCoeff[i] = new double[this->sizeTaylor[1]];
    }

    
    readFile(dataSinCoeff, "Data/sinCoeff.txt", this->sizeTaylor);
    readFile(dataCosCoeff, "Data/cosCoeff.txt", this->sizeTaylor);
    for(int i=0; i < this->sizeTaylor[0]; i++){
      for(int j=0; j < this->sizeTaylor[1]; j++){
        this->sinCoeff[i][j] = fixedPoint(dataSinCoeff[i][j], 24, 24, ALICE);
        this->cosCoeff[i][j] = fixedPoint(dataCosCoeff[i][j], 24, 24, ALICE);
      }
    }

    getFileSize("Data/yd.txt", this->sizeyd);
    double **datayd;
    datayd = new double*[this->sizeyd[0]];
    this->yd = new fixedPoint*[this->sizeyd[0]];
    for(int i=0; i < this->sizeyd[0]; i++){
      this->yd[i] = new fixedPoint[this->sizeyd[1]];
      datayd[i] = new double[this->sizeyd[1]];
    }

    readFile(datayd, "Data/yd.txt", this->sizeyd);
    for(int i=0; i < this->sizeyd[0]; i++){
      for(int j=0; j < this->sizeyd[1]; j++){
        this->yd[i][j] = fixedPoint(datayd[i][j], 24, 24, ALICE);
      }
    }
  }
};

#endif
