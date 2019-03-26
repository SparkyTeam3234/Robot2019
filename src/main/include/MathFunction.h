#pragma once

class MathFunction {
  public:
    virtual double fp (double t);
    virtual double fv (double t); //Derivative of f
    virtual double fa (double t); //2nd Derivative of f
};

class ParametricFunction {
  public:
    ParametricFunction(MathFunction* fx,MathFunction* fy) {
        this->fx=fx;
        this->fy=fy;
    }
    double xp(double t) {
        return fx->fp(t);
    }
    double yp(double t) {
        return fy->fp(t);
    }
    double xv(double t) { //Derivative of x
        return fx->fv(t);
    }
    double yv(double t) { //Derivative of y
        return fy->fv(t);
    }
    double xa(double t) { //2nd Derivative of x
        return fx->fa(t);
    }
    double ya(double t) { //2nd Derivative of y
        return fy->fa(t);
    }
    double dydx(double t) { //Derivative of y/Derivative of x
        return this->yv(t)/this->xv(t);
    }
  private:
    MathFunction* fx;
    MathFunction* fy;
};
