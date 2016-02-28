// from Facebook (unmodified)
#ifndef MATRIX_H
#define MATRIX_H

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
class Matrix{
public:
    Matrix();
    Matrix(int,int);
    Matrix(const Matrix&);
    double det();
    Matrix operator+(const Matrix&) const;
    Matrix operator-(const Matrix&) const;
    Matrix operator*(int);
    Matrix operator*(const Matrix&) const;
    Matrix &operator=(const Matrix&);
    Matrix transpose();
    double getElement(int, int);
    void setElement(int, int, double);
    int getRows() const;
    int getCols() const;
    bool print(std::ostream&);
    void build(std::istream&);
    Matrix rref();
    Matrix rref(double&);
    void pivot(int&,int);
    void pivot(int&, int,double&);
    double quickDet();
    Matrix cof();
    Matrix inv();
    bool invertible();
    Matrix basisForNullSpace(); //set of vectors
    //To do:
    int* eigVals();
    bool diagonalizable();
    Matrix orthoDiagonalize();
    Matrix orthoNormalize();
    Matrix eigVector(int);
    Matrix eigVectors(); //set of vectors
    Matrix basisForSLESolutionSpace(Matrix); //set of vectors
    ~Matrix();
private:
    int rows;
    int cols;
    double **elements;
    void copy(const Matrix&);
    void deleteAll();
};

//Matrix inv(Matrix);
//Matrix rref(Matrix);
//Matrix cof(Matrix);

#endif
