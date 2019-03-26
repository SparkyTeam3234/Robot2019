#pragma once

#include "MathFunction.h"

class BezierFunction : MathFunction {
  public:
    BezierFunction(double p0,double p1,double p2,double p3) {
      this->p0=p0;
      this->p1=p1;
      this->p2=p2;
      this->p3=p3;
    }
    virtual double fp (double t) {
      double ivt=1-t;
      double ivt2=ivt*ivt;
      double t2=t*t;
      return p0*ivt2*ivt + 3*p1*t*ivt2 + 3*p2*t2*ivt + p3*t2*t;
    }
    virtual double fv (double t) {
      double t2=t*t;
      return p0*(-3*t2+6*t-3)+3*p1*(3*t2-4*t+1)+3*p2*(2*t-3*t2)+3*p3*t2;
    }
    virtual double fa (double t) {
      double t2=t*t;
      return p0*(-6*t+6)+3*p1*(6*t-4)+3*p2*(2-6*t)+6*p3*t;
    }
  private:
    double p0;
    double p1;
    double p2;
    double p3;
};