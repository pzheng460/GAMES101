#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        if (num_nodes == 0 || num_nodes == 1) // Make sure the rope has at least 3 nodes
            return;

        Vector2D subDistance = (end - start) / (num_nodes - 1); // Calculate the distance between each node
        for (int i = 0; i < num_nodes; i++)
        {
            Vector2D CurrentPosition = start + i * subDistance;
            Mass *p = new Mass(CurrentPosition, node_mass, false);
            masses.push_back(p);
            if (i > 0) {
                Spring *presentSpring = new Spring(masses[i - 1], masses[i], k);
                springs.push_back(presentSpring);
            }
        }

        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D ab = s->m2->position - s->m1->position;  // ab = b - a
            Vector2D f = s->k * (ab / ab.norm()) * (ab.norm() - s->rest_length); // f = k * (ab / |ab|) * (|ab| - rest_length)
            s->m1->forces += f; // a += f
            s->m2->forces -= f; // b -= f
        }

        float k_d = 0.01;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass; // f_g = m * g

                // TODO (Part 2): Add global damping
                m->forces += -k_d * m->velocity; // f_d = -k_d * v
                Vector2D a = m->forces / m->mass; // a = f / m

                // Euler's method(Explicit)
//                m->position += delta_t * m->velocity; // x += v * dt
//                m->velocity += delta_t * a; // v += a * dt

                // Euler's method(Semi-Implicit)
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D ab = s->m2->position - s->m1->position; // ab = b - a
            Vector2D f = s->k * (ab.unit()) * (ab.norm() - s->rest_length); // f = k * (ab / |ab|) * (|ab| - rest_length)
            s->m1->forces += f; // a += f
            s->m2->forces -= f; // b -= f
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * m->mass; // f_g = m * g
                Vector2D a = m->forces / m->mass; // a = f / m

                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
//                m->position = m->position + (m->position - m->last_position) + a * delta_t * delta_t;
                // TODO (Part 4): Add global Verlet damping
                double damping_factor = 0.00005;
                m->position = m->position + (1 - damping_factor) * (m->position - m->last_position) + a * delta_t * delta_t;

                // Update last position
                m->last_position = temp_position;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0,0);
        }
    }
}
