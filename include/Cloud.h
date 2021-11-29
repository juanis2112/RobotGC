// Change all Alice to BOB?
#ifndef CLOUD_H
#define CLOUD_H
#include "fixedPoint.h"

class Cloud {
public:
  
  fixedPoint **uk;
  fixedPoint **sinCoeff;
  fixedPoint **cosCoeff;
  fixedPoint dFLi;
  fixedPoint dFL;
  fixedPoint **yd;
  fixedPoint **error;
  int sizeuk[2];
  int sizeyk[2];
  int sizeyd[2];
  int sizeTaylor[2];

  int decimalBits = 24;
  int totalBits = 48;

  Cloud() {}

  // class receives matrices to compute the controller and the anomaly detection
  void getInputs(int *sizeTaylor, fixedPoint **sinCoeff, fixedPoint **cosCoeff, 
                 int *sizeuk, fixedPoint **yd, fixedPoint dFLi, fixedPoint dFL, int *sizeyk, int *sizeyd) {
    this->sizeuk[0] = sizeuk[0];
    this->sizeuk[1] = sizeuk[1];
    this->uk = new fixedPoint*[this->sizeuk[0]];
    for(int i=0; i < sizeuk[0]; i++){
      this->uk[i] = new fixedPoint[this->sizeuk[1]];
    }

    this->sizeyk[0] = sizeyk[0];
    this->sizeyk[1] = sizeyk[1];
    this->sizeyd[0] = sizeyd[0];
    this->sizeyd[1] = sizeyd[1];
    this->yd = new fixedPoint*[this->sizeyd[0]];
    for (int i = 0; i < this->sizeyd[0]; i++) {
      this->yd[i] = new fixedPoint[this->sizeyd[1]];
    }
    for (int i = 0; i < this->sizeyd[0]; i++) {
      for (int j = 0; j < this->sizeyd[1]; j++) {
        this->yd[i][j] = yd[i][j];
      }
    }

    

    
    this->sizeTaylor[0] = sizeTaylor[0];
    this->sizeTaylor[1] = sizeTaylor[1];
    this->sinCoeff = new fixedPoint*[this->sizeTaylor[0]];
    this->cosCoeff = new fixedPoint*[this->sizeTaylor[0]];
    for(int i=0; i < sizeTaylor[0]; i++){
      this->sinCoeff[i] = new fixedPoint[this->sizeTaylor[1]];
      this->cosCoeff[i] = new fixedPoint[this->sizeTaylor[1]];
    }
    for (int i = 0; i < this->sizeTaylor[0]; i++) {
      for (int j = 0; j < this->sizeTaylor[1]; j++) {
        this->sinCoeff[i][j] = sinCoeff[i][j];
        this->cosCoeff[i][j] = cosCoeff[i][j];
      }
    }
    this->error = new fixedPoint *[this->sizeyd[0]];
    for (int i = 0; i < this->sizeyd[0]; i++) {
      this->error[i] = new fixedPoint[this->sizeyd[1]];
    }
    this->dFLi = dFLi;
    this->dFL = dFL;
    
  }

  // Computes the multiplication between matrices A and B
  void matrixMul(fixedPoint **A, fixedPoint **B, fixedPoint **ret, int *ASize,
                 int *BSize) {
    for (int i = 0; i < ASize[0]; i++) {
      for (int j = 0; j < BSize[1]; j++) {
        ret[i][j] = (A[i][0] * B[0][j]);
        for (int k = 1; k < ASize[1]; k++) {
          ret[i][j] = ret[i][j] + (A[i][k] * B[k][j]);
        }
      }
    }
  }

  void matrixVecMul(fixedPoint **A, fixedPoint **B, fixedPoint *ret,
                    int *size) {
    for (int i = 0; i < size[0]; i++) {
      ret[i] = ((A[i][0]) * (B[0][0]));
      for (int j = 1; j < size[1]; j++) {
        ret[i] = ret[i] + ((A[i][j]) * (B[j][0]));
      }
    }
  }

  void computeSine(fixedPoint **state, fixedPoint& cosTheta, fixedPoint& sinTheta){
    fixedPoint power = state[2][0];
    cosTheta = this->cosCoeff[0][0];
    sinTheta = this->sinCoeff[0][0];
    for(int i=1; i < this->sizeTaylor[1]-1; i++){
        cosTheta = cosTheta + power*this->cosCoeff[0][i];
        sinTheta = sinTheta + power*this->sinCoeff[0][i];
        power = power*state[2][0]; // Fix. Computes one additionnal power that we do not need.
    }
    cosTheta = cosTheta + power*this->cosCoeff[0][sizeTaylor[1]-1];
    sinTheta = sinTheta + power*this->sinCoeff[0][sizeTaylor[1]-1];
    
  }
 
  // Computes control input uk
  void computeuk(fixedPoint **yk) {
    for(int i=0; i<this->sizeyd[0]; i++){
      for(int j=0; j<this->sizeyd[1]; j++){
        this->error[i][j] = yd[i][j] - yk[i][j];
      }
    }
    fixedPoint sine = fixedPoint(0, decimalBits, decimalBits, PUBLIC);
    fixedPoint cosine = fixedPoint(0, decimalBits, decimalBits, PUBLIC);
    this->computeSine(yk, cosine, sine);
    
    this->error[0][0] = this->error[0][0] - this->dFL*cosine;
    this->error[1][0] = this->error[1][0] - this->dFL*sine;

    //cout << yk[2][0].reveal<double>(ALICE) << endl;
    //cout << sine.reveal<double>(ALICE) << endl;

    this->uk[0][0] = cosine*this->error[0][0] + sine*this->error[1][0];
    this->uk[1][0] = -sine*this->error[0][0]  + cosine*this->error[1][0];
    this->uk[1][0] = this->dFLi*this->uk[1][0];
    
  }


};

#endif
