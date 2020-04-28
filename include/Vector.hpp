/*
Simple vector library, could always be expanded upon
Created by Nick Crane 
*/

#pragma once
#include <math.h>

/* Used to find the absolute value of doubles */
static double dabs(double x)
{
    if(x < 0)
        return -x;
    return x;
}

struct vec2d 
{
    double i, j;

    vec2d() {i = 0; j = 0;}
    vec2d(double x, double y) {i = x; j = y;}

    ~vec2d() {}

    inline vec2d operator+(const vec2d& other) {return vec2d(i + other.i, j + other.j);}
    inline vec2d operator-(const vec2d& other) {return vec2d(i - other.i, j - other.j);}
    inline void operator+=(const vec2d& other) {i += other.i; j += other.j;}
    inline void operator-=(const vec2d& other) {i -= other.i; j -= other.j;}
    inline vec2d operator*(const double& x) {return vec2d(i * x, j * x);}
    inline double mag() {return sqrt(i*i + j*j);}
    inline double mag2() {return i*i + j*j;}
};

struct vec3d 
{
    double i, j, k;

    vec3d() {i = 0; j = 0; k = 0;}
    vec3d(double x, double y, double z) {i = x; j = y; k = z;}

    ~vec3d() {}

    inline vec3d operator+(const vec3d& other) {return vec3d(i + other.i, j + other.j, k + other.k);}
    inline vec3d operator-(const vec3d& other) {return vec3d(i - other.i, j - other.j, k - other.k);}
    inline void operator+=(const vec3d& other) {i += other.i; j += other.j; k += other.k;}
    inline void operator-=(const vec3d& other) {i -= other.i; j -= other.j; k -= other.k;}
    inline vec3d operator*(const double& x) {return vec3d(i * x, j * x, k * x);}
    inline bool operator==(const vec3d& other) {return i==other.i && j==other.j && k==other.k;}
    // inline double operator*(const vec3d& other) {return i * other.i + j * other.j + k * other.k;}
    inline double mag() {return sqrt(i*i + j*j + k*k);}
    inline double mag2() {return i*i + j*j + k*k;}
};
