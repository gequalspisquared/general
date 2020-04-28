/* 
A particle based N-Body gas sim
Created by Nick Crane
*/

/* 
Assumes neutral particles, uses Van der Waals to approximate forces
Can be changed to include gravity
*/

// #include <future> // async????
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include "../include/PhysicsObjects3D.hpp"

#define RUNANIM 1 // used to determine whether to run anim or not
#define RANDOM rand()/RAND_MAX // used for initial conditions

// uses Van der Waals approximation 
void UpdateParticleVelocities(PARTICLE** pArray, int N, float dt, double r8, double r14)
{
    vec3d f, u; // force vector and direction vector
    double mag2, mag8, mag14;

    for(int i = 0; i < N - 1; i++) 
    {
        for (int j = i + 1; j < N; j++)
        {
            u  = pArray[j]->s - pArray[i]->s;
            mag2 = u.mag2();
            mag8 = mag2 * mag2 * mag2 * mag2; mag14 = mag8 * mag8 / mag2;
            f = u * ( -24 * ( ( 2 * r14 / mag14 ) - ( r8 / mag8 ) ) );
            pArray[i]->v += f * ( dt / pArray[i]->m );
            pArray[j]->v -= f * ( dt / pArray[j]->m );
        }
    }
}

/* // trying to use std::async to thread the velocity update
void UpdateParticleAccelerations(PARTICLE** pArray, int i, int N, float dt, double r7, double r13)
{
    vec3d f, u;
    double mag, mag2, mag7, mag13;

    for (int j = i + 1; j < N; j++)
    {
        u  = pArray[i]->s - pArray[j]->s;
        mag = u.mag();
        mag2 = mag * mag; mag7 = mag2 * mag2 * mag2 * mag; mag13 = mag7 * mag7 / mag;
        f = u * (24 * ((r13 / mag13) - (r7 / mag7)));
        pArray[i]->a += f * (dt / pArray[i]->m);
        pArray[j]->a -= f * (dt / pArray[j]->m);
    }
}
*/

// uses the leapfrog algorithm
void UpdateParticlePositions(PARTICLE** pArray, int N, float dt)
{
    for (int i = 0; i < N; i++)
    {
        pArray[i]->s += pArray[i]->v * (dt / 2);
    }
}

// checks to see if particles will leave box
void FixParticlePosition(PARTICLE** pArray, int N, float dt, int L, double& momentum)
{
    double nx, ny, nz;
    for (int i = 0; i < N; i++)
    {
        nx = dabs(pArray[i]->s.i + pArray[i]->v.i * dt);
        ny = dabs(pArray[i]->s.j + pArray[i]->v.j * dt);
        nz = dabs(pArray[i]->s.k + pArray[i]->v.k * dt);

        if (nx > L / 2)
        {
            pArray[i]->v.i *= -1;
            momentum += dabs(pArray[i]->p().i) * 2;
        }
        if (ny > L / 2)
        {
            pArray[i]->v.j *= -1;
            momentum += dabs(pArray[i]->p().j) * 2;
        }
        if (nz > L / 2)
        {
            pArray[i]->v.k *= -1;
            momentum += dabs(pArray[i]->p().k) * 2;
        }
    }
}

void CalculatePotential(PARTICLE** pArray, int N, double r6, double r12, double& potential)
{
    vec3d u; // direction vector
    double mag, mag2, mag6, mag12;
    for (int i = 0; i < N - 1; i++)
    {
        for (int j = i + 1; j < N; j++)
        {
            u  = pArray[j]->s - pArray[i]->s;
            mag = u.mag();
            mag2 = mag * mag; mag6 = mag2 * mag2 * mag2; mag12 = mag6 * mag6;
            potential += 4 * ( ( r12 / mag12 ) - ( r6 / mag6 ) );
        }
    }
    printf("pot = %f\n", potential);
}

void UpdateInfo(PARTICLE** pArray, int N, float dt, int L, int ticks, double& momentum, double& potential)
{
    double PV = L * L * L * momentum / ( dt * ticks * L * L * 6 );
    double KE = 0, TEMP = 0;

    for (int i = 0; i < N; i++) // calculate kinetic energy
    {
        double v = pArray[i]->v.mag();
        KE += pArray[i]->ke();
    }
    TEMP = 2 * KE / 3;

    printf("!kT = %f, KE = %f, PV = %f, U = %f, U+KE = %f\n", TEMP, KE, PV, potential, potential + KE);

    momentum = potential = 0;
}

int main() 
{
    // parameters
    int N = 1000;
    static int L = 100; // length of bounding box
    static float dt = 0.01f;
    static const double r1 = 1; // critical distance where force is 0
    int ticksPerInfoUpdate = 100;
    int ticksPerFrame = 10;

    // particle properties
    double mass = 1, radius = 1;
    double initialv = 1;

    // derived quantities (do not touch)
    static const double r2 = r1 * r1;
    static const double r6 = r2 * r2 * r2;
    static const double r7 = r6 * r1;
    static const double r8 = r6 * r2;
    static const double r12 = r6 * r6;
    static const double r13 = r12 * r1;
    static const double r14 = r13 * r1;

    // variables used by program (do not touch)
    double momentum = 0, potential = 0;
    float time = 0.0f;
    int tick = 0;

    PARTICLE** particles = new PARTICLE*[N] { 0 };

    for (int i = 0; i < N; i++)
    {
        particles[i] = new PARTICLE(
            mass, radius, 
            vec3d((double) RANDOM * L - L / 2, (double) RANDOM * L - L / 2, (double) RANDOM * L - L / 2),
            vec3d((2 * RANDOM - 1) * initialv, (2 * RANDOM - 1) * initialv, (2 * RANDOM - 1) * initialv),
            vec3d(0, 0, 0) );
    }

    while (true)
    {
        // trying to implement std::async to thread the velocity update (most expensive function)
        // for (int i = 0; i < N - 1; i++)
        // {
        //     p_Futures.push_back(std::async(std::launch::async, UpdateParticleVelocities, particles, i, N, dt, r8, r14));
        // }
        
        // leapfrog algorithm
        UpdateParticlePositions(particles, N, dt);
        UpdateParticleVelocities(particles, N, dt, r8, r14);
        UpdateParticlePositions(particles, N, dt);
        FixParticlePosition(particles, N, dt, L, momentum);
        time += dt; tick++;

        if (tick % ticksPerInfoUpdate == 0)
        {
            CalculatePotential(particles, N, r6, r12, potential);
            UpdateInfo(particles, N, dt, L, ticksPerInfoUpdate, momentum, potential);
            printf("!v = %f, %f, %f\n", particles[0]->v.i, particles[0]->v.j, particles[0]->v.k);
        }

#if RUNANIM==1
        if (tick % ticksPerFrame == 0)
        {
            for(int i = 0; i < N; i++)
                {
                    printf("c3 %f %f %f 1\n", particles[i]->s.i, particles[i]->s.j, particles[i]->s.k);
                }
                printf("F\n");
        }
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
