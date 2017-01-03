"""
Created on 20.12.2016

contains different functions for the flocking pattern
based on paper / algorithms by Olfati-Saber

@author: kaiser
"""
from __future__ import division  # ensures floating point divisions
from geometry_msgs.msg import Vector3
import numpy as np
import calc


def agent_velocity(p1, p2):
    """
    :param p1: gradient 1 (soMessage)
    :param p2: gradient 2 (soMessage)
    :return: current agent velocity
    """
    v = calc.delta_vector(p1.p, p2.p)

    # delta t in secs
    dt = p1.header.stamp.secs - p2.header.stamp.secs
    dt += (p1.header.stamp.nsecs - p2.header.stamp.nsecs) / 1000000000.0

    if dt == 0.0:
        return Vector3()

    # velocity = delta distance / delta time
    v.x /= dt
    v.y /= dt
    v.z /= dt

    return v


def gradient_based(neighbors, agent, epsilon, a, b, avoidance_distance, view_distance, h):
    """
    :param neighbors: array of neighbor positions of agent i (tuple position - velocity)
    :param agent: agent under consideration
    :param epsilon: sigma norm parameter (0,1)
    :param a: action function parameter
    :param b: action function parameter
    :param view_distance: interaction range of robot
    :param h: parameter (0,1) specifying boundaries of bump function
    :param avoidance_distance: scale (desired distance between agents)
    :return: gradient based term for flocking
    """
    v = Vector3()

    for q in neighbors:
        nij = vector_btw_agents(agent.p, q.p, epsilon)
        m = action_function(q.p, agent.p, view_distance, epsilon, a, b, avoidance_distance, h)
        v.x += m * nij.x
        v.y += m * nij.y
        v.z += m * nij.z

    return v


def velocity_consensus(neighbors, agent, epsilon, r, h):
    """
    :param neighbors: array of neighbors of agent i (tuple position - velocity)
    :param agent: agent under consideration (position and velocity)
    :param epsilon_r: sigma norm parameter (0,1) of r
    :param epsilon_z: sigma norm parameter of z
    :param r: view distance / interaction range
    :param h: parameter specifying boundaries of bump function
    :return: velocity consensus term for flocking
    """
    v = Vector3()

    for q in neighbors:
        # needs velocity - delta velocity
        dp = calc.delta_vector(q.v, agent.v)
        # needs position
        aij = adjacency_matrix(q.p, agent.p, epsilon, r, h)

        v.x += aij * dp.x
        v.y += aij * dp.y
        v.z += aij * dp.z

    return v


def action_function(qj, qi, r, epsilon, a, b, avoidance_distance, h):
    """
    :param qj: Positin neighbor Vector3
    :param qi: Position agent Vector3
    :param r: view_distance / interaction range
    :param epsilon: parameter of sigma norm (0,1)
    :param a: parameter
    :param b: parameter
                0 < a <= b; c = |a-b|/np.sqrt(4ab)
    :param h: parameter (0,1) specifying boundaries of bump function
    :param avoidance_distance: distance between agents
    :return:
    """
    dq = calc.delta_vector(qj, qi)
    z = sigma_norm(epsilon, dq)
    r_alpha = sigma_norm_f(epsilon, r)
    d_alpha = sigma_norm_f(epsilon, avoidance_distance)

    # calculate parameter c for action function
    c = np.abs(a-b)/np.sqrt(4*a*b)

    z_phi = z - d_alpha
    sigma = (z_phi + c) / (np.sqrt(1+np.square(z_phi+c)))
    phi = 0.5 * ((a+b)*sigma+(a-b))
    phi_alpha = bump_function(z/r_alpha, h)*phi

    return phi_alpha


def sigma_norm(epsilon, z):
    """
    :param epsilon: fixed parameter (0, 1)
    :param z: Vector3
    :return: sigma norm of z
    """
    # Euclidean norm of vector
    z = calc.vector_length(z)
    return (1/epsilon)*(np.sqrt(1+epsilon * np.square(z))-1)


def sigma_norm_f(epsilon, z):
    """
    :param epsilon: fixed parameter (0, 1)
    :param z: float
    :return: sigma norm of z
    """
    # Euclidean norm of vector
    return (1/epsilon)*(np.sqrt(1+epsilon * np.square(z))-1)


def bump_function(z, h):
    """
    scalar function varying between 0 and 1
    to construct smooth potential functions
    :param z: input parameter of bump function
    :param h: parameter (0,1) specifying boundaries
    :return: value between 0 and 1
    """
    if 0 <= z < h:
        return 1.0
    elif h <= z <= 1:
        return 0.5 * (1.0 + np.cos(np.pi * ( (z - h)/(1.0 - h))))
    else:
        return 0.0


def adjacency_matrix(qj, qi, epsilon, r, h):
    """
    :param qj: Position neighbor
    :param qi: Position agent
    :param epsilon_r: parameter of sigma norm (0,1) of r
    :param epsilon_z: parameter of sigma norm of z
    :param r: view distance / interaction range
    :param h: parameter (0,1) of bump_function
    :return: adjacency matrix element aij
    """
    dq = calc.delta_vector(qi, qj)
    z = sigma_norm(epsilon, dq)
    r_alpha = sigma_norm_f(epsilon, r)

    return bump_function(z/r_alpha, h)


def vector_btw_agents(qi, qj, epsilon):
    """
    :param qi: agent under consideration
    :param qj: neighbor
    :param epsilon: fixed parameter (0,1) of sigma_norm
    :return: vector along the line btw qj and qi
    """
    n = Vector3()
    # dq = qj - qi
    dq = calc.delta_vector(qj, qi)
    # denominator of calculation
    denom = np.sqrt(1 + epsilon * np.square(calc.vector_length(dq)))

    n.x = dq.x / denom
    n.y = dq.y / denom
    n.z = dq.z / denom

    return n


def flocking_vector(neighbors, agent, epsilon, a, b, repulsion_radius, view_distance, h):
    """
    :param neighbors:
    :param agent:
    :param epsilon:
    :param a:
    :param b:
    :param repulsion_radius:
    :param view_distance:
    :param h:
    :return: vector of steering force
    """
    grad = gradient_based(neighbors, agent, epsilon, a, b, repulsion_radius, view_distance, h)

    vel = velocity_consensus(neighbors, agent, epsilon, view_distance, h)

    return calc.add_vectors(grad, vel)