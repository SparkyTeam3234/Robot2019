#pragma once

inline double cabs(double arg) {
	return arg < 0 ? -arg : arg;
}
inline double cabs(int arg) {
	return arg < 0 ? -arg : arg;
}
inline double cmax(double arg1, double arg2) {
	return arg1 < arg2 ? arg2 : arg1;
}
inline double cmin(double arg1, double arg2) {
	return arg1 > arg2 ? arg2 : arg1;
}
inline double cmax(int arg1, int arg2) {
	return arg1 < arg2 ? arg2 : arg1;
}
inline double cmin(int arg1, int arg2) {
	return arg1 > arg2 ? arg2 : arg1;
}
