/* 
A particle based N-Body gas sim
Created by Nick Crane
*/

/* 
Assumes neutral particles, uses Van der Waals to approximate forces
Can be changed to include gravity
*/


#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include "../include/PhysicsObjects3D.hpp"

#define RUNANIM 0 // used to determine whether to run anim or not
#define RANDOM rand()/RAND_MAX // used for initial conditions

// uses Van der Waals approximation 
void UpdateParticleAccelerations(PARTICLE** pArray, int N, float dt, double r7, double r13)
{
    vec3d f, u; // force vector and direction vector
    double mag, mag2, mag7, mag13;

    for(int i = 0; i < N - 1; i++) 
    {
        for (int j = i + 1; j < N; j++)
        {
            u  = pArray[j]->s - pArray[i]->s;
            mag = u.mag();
            mag2 = mag * mag; mag7 = mag2 * mag2 * mag2 * mag; mag13 = mag7 * mag7 / mag;
            f = u * ( 24 * ( ( r7 / mag7 ) - ( r13 / mag13 ) ) );
            pArray[i]->a += f * ( dt / pArray[i]->m );
            pArray[j]->a -= f * ( dt / pArray[j]->m );
        }
    }
}



/*
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
void UpdateParticleKinematics(PARTICLE** pArray, int N, float dt)
{
    for (int i = 0; i < N; i++)
    {
        pArray[i]->s += pArray[i]->v * (dt / 2);
        pArray[i]->v += pArray[i]->a * dt;
        pArray[i]->s += pArray[i]->v * (dt / 2);
    }
}

// checks to see if particles will leave box
void FixParticlePosition(PARTICLE** pArray, int N, float dt, int L, double& momentum)
{
    double nx, ny, nz;
    for (int i = 0; i < N; i++)
    {
        nx = pArray[i]->s.i + pArray[i]->v.i * dt;
        ny = pArray[i]->s.j + pArray[i]->v.j * dt;
        nz = pArray[i]->s.k + pArray[i]->v.k * dt;

        if (nx > L || nx < 0)
        {
            pArray[i]->v.i *= -1;
            momentum += dabs(pArray[i]->p().i) * 2;
        }
        if (ny > L || ny < 0)
        {
            pArray[i]->v.j *= -1;
            momentum += dabs(pArray[i]->p().j) * 2;
        }
        if (nz > L || nz < 0)
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
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (j != i)
            {
                u  = pArray[j]->s - pArray[i]->s;
                mag = u.mag();
                mag2 = mag * mag; mag6 = mag2 * mag2 * mag2; mag12 = mag6 * mag6;
                potential += 24 * ( ( 13 * r12 / mag12 ) - ( 7 * r6 / mag6 ) );
            }
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

    printf(" kT = %f, KE = %f, PV = %f, U = %f, U+KE = %f\n", TEMP, KE, PV, potential, potential + KE);

    momentum = potential = 0;
}

int main() 
{
    int N = 100;
    static int L = 10; // length of bounding box
    float time = 0.0f;
    static float dt = 0.0001f;
    int tick = 0;
    double momentum = 0;
    static const double r1 = 0.5; // critical distance where force is 0
    int ticksPerInfoUpdate = 100;
    double potential = 0;

    // initial conditions
    double mass = 1, radius = 1;
    double initialv = 10;

    // derived quantities
    static const double r2 = r1 * r1;
    static const double r6 = r2 * r2 * r2;
    static const double r7 = r6 * r1;
    static const double r12 = r6 * r6;
    static const double r13 = r12 * r1;
    static const int V = L * L * L;

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


        UpdateParticleAccelerations(particles, N, dt, r7, r13);
        UpdateParticleKinematics(particles, N, dt);
        FixParticlePosition(particles, N, dt, L, momentum);
        time += dt; tick++;

        if (tick % ticksPerInfoUpdate == 0)
        {
            CalculatePotential(particles, N, r6, r12, potential);
            UpdateInfo(particles, N, dt, L, ticksPerInfoUpdate, momentum, potential);
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
