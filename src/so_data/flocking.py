"""
Created on 20.12.2016

contains different functions for the flocking pattern
based on algorithm by Olfati-Saber

@author: kaiser
"""

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

    # delta t
    dt = p1.stamp - p2.stamp

    # velocity = delta distance / delta time
    v.x /= dt.secs
    v.y /= dt.secs
    v.z /= dt.secs

    return v


def gradient_based(neighbors, agent, epsilon, a, b):
    """
    :param neighbors: array of neighbor positions of agent i (tuple position - velocity)
    :param agent: agent under consideration
    :param epsilon: sigma norm parameter (0,1)
    :param a: action function parameter
    :param b: action function parameter
    :return: gradient based term for flocking
    """
    v = Vector3()

    for q in neighbors:
        nij = vector_btw_agents(agent.p, q.p, epsilon)
        m = action_function(q.p, agent.p, epsilon, a, b)
        v.x += m * nij.x
        v.y += m * nij.y
        v.z += m * nij.z

    return v


def velocity_consensus(neighbors, agent, epsilon, r):
    """
    :param neighbors: array of neighbors of agent i (tuple position - velocity)
    :param agent: agent under consideration (position and velocity)
    :param epsilon: sigma norm parameter (0,1)
    :param r: view distance / interaction range
    :return: velocity consensus term for flocking
    """
    v = Vector3()

    for q in neighbors:
        #needs velocity
        dp = calc.delta_vector(q.v, agent.v)
        # needs position
        aij = adjacency_matrix(q.p, agent.p, epsilon, r)

        v.x += aij * dp.x
        v.y += aij * dp.y
        v.z += aij * dp.z

    return v


def action_function(qj, qi, r, epsilon, a, b):
    """
    :param qj: Positin neighbor Vector3
    :param qi: Position agent Vector3
    :param r: view_distance / interaction range
    :param epsilon: parameter of sigma norm (0,1)
    :param a: parameter
    :param b: parameter
    0 < a <= b; c = |a-b|/np.sqrt(4ab)
    :return:
    """

    dq = calc.delta_vector(qj, qi)
    z = sigma_norm(epsilon, dq)
    r_alpha = sigma_norm(epsilon, r)

    # calculate parameter c for action function
    c = np.abs(a-b)/np.sqrt(4*a*b)

    phi = 0.5 * ((a+b)*(z / np.sqrt(1+np.square(z)))*(z+c)+(a-b))
    phi_alpha = bump_function(z/r_alpha)*phi

    return phi_alpha


def sigma_norm(epsilon, z):
    """
    :param epsilon: parameter, > 0
    :param z: Vector3
    :return: sigma norm of z
    """
    # Euclidean norm of vector
    z = calc.vector_length(z)
    return (1/epsilon)*(np.sqrt(1+epsilon * np.square(z))-1)


def bump_function(z, h):
    """
    scalar function varying between 0 and 1
    to construct smooth potential functions
    :param z: input parameter of bump function
    :param h: parameter element (0,1)
    :return:
    """
    if 0 <= z < h:
        return 1
    elif h <= z <= 1:
        return 0.5 * (1 + np.cos(np.pi * ( (z - h)/(1 - h))))
    else:
        return 0


def adjacency_matrix(qj, qi, epsilon, r): #TODO
    """
    :param qj: Position neighbor
    :param qi: Position agent
    :param epsilon: parameter of sigma norm (0,1)
    :param r: view distance / interaction range
    :return: adjacency matrix element aij
    """
    dq = calc.delta_vector(qj, qi)
    z = sigma_norm(epsilon, dq)
    r_alpha = sigma_norm(epsilon, r)

    return bump_function(z/r_alpha)



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

    n.x = dq.x / np.sqrt(1 + epsilon * np.square(calc.vector_length(dq)))
    n.y = dq.y / np.sqrt(1 + epsilon * np.square(calc.vector_length(dq)))
    n.z = dq.z / np.sqrt(1 + epsilon * np.square(calc.vector_length(dq)))

    return n
