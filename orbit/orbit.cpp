/* 
A N-Body sim
Created by Nick Crane
*/


#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include "../include/PhysicsObjects3D.hpp"

#define RUNANIM 0 // used to determine whether to run anim or not
#define RANDOM rand()/RAND_MAX // used for initial conditions

// uses Van der Waals approximation 
void UpdateParticleAccelerations(PARTICLE** pArray, int N, float dt)
{
    vec3d f, u; // force vector and direction vector
    double mag;

    for(int i = 0; i < N - 1; i++) 
    {
        for (int j = i + 1; j < N; j++)
        {
            u  = pArray[j]->s - pArray[i]->s;
            mag = u.mag();
            pArray[i]->a += f * ( dt / pArray[i]->m );
            pArray[j]->a -= f * ( dt / pArray[j]->m );
        }
    }
}

// uses the leapfrog algorithm
void UpdateParticleKinematics(PARTICLE** pArray, int N, float dt)
{
    for (int i = 0; i < N; i++)
    {
        pArray[i]->s += pArray[i]->v * (dt / 2);
        pArray[i]->v += pArray[i]->a * dt;
        pArray[i]->s += pArray[i]->v * (dt / 2);
    }
}

void CalculatePotential(PARTICLE** pArray, int N, double& potential)
{
    vec3d u; // direction vector
    double mag, mag2, mag6, mag12;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (j != i)
            {
                u  = pArray[j]->s - pArray[i]->s;
                mag = u.mag();
                potential += 
            }
        }
    }
    printf("pot = %f\n", potential);
}

void UpdateInfo(PARTICLE** pArray, int N, float dt, int ticks, double& potential)
{
    double KE = 0;

    for (int i = 0; i < N; i++) // calculate kinetic energy
    {
        double v = pArray[i]->v.mag();
        KE += pArray[i]->ke();
    }

    printf(" kT = %f, KE = %f, PV = %f, U = %f, U+KE = %f\n", KE, potential, potential + KE);

    potential = 0;
}

int main() 
{
    int N = 100;
    float time = 0.0f;
    static float dt = 0.0001f;
    int tick = 0;
    int ticksPerInfoUpdate = 100;
    double potential = 0;

    // initial conditions
    double mass = 1, radius = 1;
    double initialv = 10;

    PARTICLE** particles = new PARTICLE*[N] { 0 };

    for (int i = 0; i < N; i++)
    {
        particles[i] = new PARTICLE(
            mass, radius, 
            vec3d((double) RANDOM * L, (double) RANDOM * L, (double) RANDOM * L),
            vec3d((2 * RANDOM - 1) * initialv, (2 * RANDOM - 1) * initialv, (2 * RANDOM - 1) * initialv),
            vec3d(0, 0, 0) );
    }

    while (time < 0.1)
    {
        // for (int i = 0; i < N - 1; i++)
        // {
        //     p_Futures.push_back(std::async(std::launch::async, UpdateParticleAccelerations, particles, i, N, dt, r7, r13));
        // }


        UpdateParticleAccelerations(particles, N, dt);
        UpdateParticleKinematics(particles, N, dt);
        time += dt; tick++;

        if (tick % ticksPerInfoUpdate == 0)
        {
            CalculatePotential(particles, N, potential);
            UpdateInfo(particles, N, dt, ticksPerInfoUpdate, potential);
        }

#if RUNANIM==1
        for(int i = 0; i < N; i++)
            {
                printf("c3 %f %f %f 1\n", particles[i]->s.i, particles[i]->s.j, particles[i]->s.k);
            }
            printf("F\n");
#endif
    }

    for (int i = 0; i < N; i++)
    {
        delete particles[i];
    }
    delete[] particles;

#if RUNANIM==1
    printf("Q\n");
#endif

    return 0;
}
