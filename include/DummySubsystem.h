#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H
#include "fixedPoint.h"
// CHange all Alice to BOB?
class subSystem {
public:
  // declares the constants for the system.
  // unlike the subsystem class, this class does not have
  // plaintexts
  fixedPoint **x0;
  fixedPoint **uk;
  int sizeuk[2];
  int sizeyk[2];
  int sizeyd[2];
  fixedPoint **yk;
  fixedPoint **yd;
  fixedPoint dFLi;
  fixedPoint dFL;

  fixedPoint **sinCoeff;
  fixedPoint **cosCoeff;
  int sizeTaylor[2];

  subSystem() {}

  

  // Reveals the value of u[k] to the system
  void measureState(fixedPoint **uk) {
    for (int i = 0; i < this->sizeuk[0]; i++) {
      for (int j = 0; j < this->sizeuk[1]; j++) {
        uk[i][j].reveal<double>(ALICE);
      }
    }
  }

  // Creates z[k] to be used by the emp library.
  // Value is set to zero since the cloud party does not have the plaintext of z[k]
  void computeyk() {
    for (int i = 0; i < this->sizeyk[0]; i++) {
      for (int j = 0; j < this->sizeyk[1]; j++) {
        this->yk[i][j] = fixedPoint(0, 24, 24, ALICE);
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
        ret[i][j] = (A[i][0] * B[0][j]);
        for (int k = 1; k < ASize[1]; k++) {
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
      ret[i] = ((A[i][0]) * (B[0][0]));
      for (int j = 1; j < size[1]; j++) {
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

  void initializeData(){
    this->dFLi = fixedPoint(0, 24, 24, ALICE);
    this->dFL = fixedPoint(0, 24, 24, ALICE);
    this->sizeuk[0] = 2;
    this->sizeuk[1] = 1;


    getFileSize("Data/x0.txt", this->sizeyk);
    this->yk = new fixedPoint*[this->sizeyk[0]];
    for(int i = 0; i<this->sizeyk[0]; i++){
      this->yk[i] = new fixedPoint[this->sizeyk[1]];
    }


    for (int i = 0; i < this->sizeyk[0]; i++) {
      for (int j = 0; j < this->sizeyk[1]; j++) {
        this->yk[i][j] = fixedPoint(0, 24, 24, ALICE);
      }
    }

    getFileSize("Data/sinCoeff.txt", this->sizeTaylor);
    this->sinCoeff = new fixedPoint*[this->sizeTaylor[0]];
    this->cosCoeff = new fixedPoint*[this->sizeTaylor[0]];
    for(int i=0; i < this->sizeTaylor[0]; i++){
      this->sinCoeff[i] = new fixedPoint[this->sizeTaylor[1]];
      this->cosCoeff[i] = new fixedPoint[this->sizeTaylor[1]];
    }

    for(int i=0; i < this->sizeTaylor[0]; i++){
      for(int j=0; j < this->sizeTaylor[1]; j++){
        this->sinCoeff[i][j] = fixedPoint(0, 24, 24, ALICE);
        this->cosCoeff[i][j] = fixedPoint(0, 24, 24, ALICE);
      }
    }

    getFileSize("Data/yd.txt", this->sizeyd);
    this->yd = new fixedPoint*[this->sizeyd[0]];
    for(int i=0; i < this->sizeyd[0]; i++){
      this->yd[i] = new fixedPoint[this->sizeyd[1]];
    }

    for(int i=0; i < this->sizeyd[0]; i++){
      for(int j=0; j < this->sizeyd[1]; j++){
        this->yd[i][j] = fixedPoint(0, 24, 24, ALICE);
      }
    }
  }

};

#endif
