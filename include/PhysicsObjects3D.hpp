#pragma once
#include "PhysicsConstants.hpp"
#include "Vector.hpp"
#include <future>
#include <vector>

class PARTICLE 
{
public:
    double m, r;
    vec3d s, v, a;

    vec3d p() 
    {
        return v * m;
    }

    double ke()
    {
        double vm = v.mag();
        return 0.5 * m * vm * vm;
    }

    PARTICLE()
    {
        m = r = 1;
        s = v = a = vec3d(0, 0, 0);
    }

    PARTICLE(double mass, double rad, vec3d pos, vec3d vel, vec3d acc)
    {
        m = mass; r = rad; s = pos; v = vel; a = acc;
    }

    ~PARTICLE() {}
};

class BODY : public PARTICLE 
{
    double mu = univG * m;
};

class CHARGEDPARTICLE : public PARTICLE
{
    double q;

    CHARGEDPARTICLE() 
    {
        m = r = 1; q = 0;
        s = v = a = vec3d(0, 0, 0);
    }

    CHARGEDPARTICLE(double mass, double rad, double chg, vec3d pos, vec3d vel, vec3d acc)
    {
        m = mass; r = rad; q = chg; s = pos; v = vel; a = acc;
    }

};

std::vector<std::future<void>> p_Futures;