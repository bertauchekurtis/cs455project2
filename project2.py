# kurtis bertauche
# cs 455
# project 1

from math import sqrt, cos, pi
import numpy as np
import matplotlib.pyplot as plt
import os

C_1_ALPHA = 30
C_2_ALPHA = 2 * sqrt(C_1_ALPHA)
C_1_BETA = 150
C_2_BETA = 2 * sqrt(C_1_BETA)
C_MT_1 = 1.1
C_MT_2 = 2 * sqrt(C_MT_1)
Q_MT = (150, 150)
EPSILON = 0.1
H = 0.2
A = B = 5
C = (abs(A - B) / sqrt(4 * A * B))
NUM_NODES = 150
DIMENSIONS = 2
D = 15
K = 1.2
R = K * D
D_PRIME = 15
R_PRIME = K * D_PRIME
DELTA_T = 0.009
RANDOM_SEED = 28
np.random.seed(28)

def main():

    try:
        os.mkdir("./caseOne")
    except:
        pass
    try:
        os.mkdir("./caseTwo")
    except:
        pass


    ################################################################
    # CASE 1 
    ################################################################

    # nodePositions = np.random.randint(low = 0, high = 70, size = (DIMENSIONS, NUM_NODES))
    # nodeVelocities = np.zeros((DIMENSIONS, NUM_NODES))
    # nodeAccelerations = np.zeros((DIMENSIONS, NUM_NODES))
    # allPositions = nodePositions.copy().reshape(DIMENSIONS, NUM_NODES, 1)
    # allNodeVelocities = nodeVelocities.copy().reshape(DIMENSIONS, NUM_NODES, 1)
    # obstacleCenters = np.array(([100, 25], [150, 30], [200, 25]))
    # obstacleRadii = np.array((15, 25, 30))
    # print(obstacleCenters)

    # plotNodesAndObstaclesAndSave(nodePositions, "caseOne/caseOne_start.png", obstacleRadii, obstacleCenters)

    # for i in range(0, 600):
    #     nodePositions, nodeVelocities, nodeAccelerations = updateAlgo3(nodePositions, nodeVelocities, (250, 25), (0, 0), obstacleCenters, obstacleRadii)
    #     if (i + 1) % 100 == 0:
    #         plotNodesAndObstaclesAndSave(nodePositions, "caseOne/caseOne" + str(i + 1) + ".png", obstacleRadii, obstacleCenters)
    #     print("FINISHED ITERATION: ", i)
    #     allPositions = np.concatenate((allPositions, nodePositions.copy()[:, :, np.newaxis]), axis = 2)
    #     allNodeVelocities = np.concatenate((allNodeVelocities, nodeVelocities.copy()[:, :, np.newaxis]), axis = 2)

    # plotNodesAndObstaclesAndSave(nodePositions, "caseOne/caseOne_end.png", obstacleRadii, obstacleCenters)
    # plotAndSaveNodeTrajectories(allPositions, "caseOne/caseOne_trajectory.png", "Fragmentation")
    # plotAndSaveNodeVelocities(allNodeVelocities, "caseOne/caseOne_individual_velocity.png", "caseOne/caseOne_all_velcoity.png", "Fragmentation")
    # plotAndSaveConnectivity(allPositions, "caseOne/caseOne_connectivity.png", "Fragmentation")

    ################################################################
    # CASE 2 - SIN WAVE
    ################################################################

    nodePositions = np.random.randint(low = 0, high = 70, size = (DIMENSIONS, NUM_NODES))
    nodeVelocities = np.zeros((DIMENSIONS, NUM_NODES))
    nodeAccelerations = np.zeros((DIMENSIONS, NUM_NODES))
    allPositions = nodePositions.copy().reshape(DIMENSIONS, NUM_NODES, 1)
    allNodeVelocities = nodeVelocities.copy().reshape(DIMENSIONS, NUM_NODES, 1)
    obstacleCenters = np.array(([100, 25], [150, 30], [200, 25]))
    obstacleRadii = np.array((15, 25, 30))
    allXCenters = []
    allYCenters = []
    gammaX = np.arange(40, 250, ((250 - 40) / 600))
    gammaY = (np.sin(2 * np.pi * gammaX / 600) * 75) + 25

    for i in range(0, 600):
        if i == 0:
            nodePositions, nodeVelocities, nodeAccelerations = updateAlgo3(nodePositions, nodeVelocities, (gammaX[i], gammaY[i]), (0, 0), obstacleCenters, obstacleRadii)
        else:
            nodePositions, nodeVelocities, nodeAccelerations = updateAlgo3(nodePositions, nodeVelocities, (gammaX[i], gammaY[i]), ((gammaX[i] - gammaX[i - 1])/DELTA_T, (gammaY[i] - gammaY[i - 1])/DELTA_T), obstacleCenters, obstacleRadii)
            
        allPositions = np.concatenate((allPositions, nodePositions.copy()[:, :, np.newaxis]), axis = 2)
        allNodeVelocities = np.concatenate((allNodeVelocities, nodeVelocities.copy()[:, :, np.newaxis]), axis = 2)

        thisXCenter = nodePositions[0, :].copy()
        thisXCenter = np.sum(thisXCenter)
        thisXCenter /= NUM_NODES
        allXCenters.append(thisXCenter)
        thisYCenter = nodePositions[1, :].copy()
        thisYCenter = np.sum(thisYCenter)
        thisYCenter /= NUM_NODES
        allYCenters.append(thisYCenter)
            

        if ((i + 1) % 100 == 0) or (i == 0):
            plt.clf()
            plt.plot(gammaX, gammaY, color = "red", label = "Gamma Agent")
            plt.plot(allXCenters, allYCenters, color = "black", label = "Center of Mass")
            plotNodes(nodePositions)
            plt.scatter(gammaX[i], gammaY[i], color = "green", zorder = 100, label = "Target")
            plt.legend(bbox_to_anchor = (1, -0.3), borderaxespad = 0)
            plt.title("Case 3: Sin Wave")
            ax = plt.gcf().gca()
            for j in range(0, len(obstacleRadii)):
                thisCircle = plt.Circle((obstacleCenters[j][0], obstacleCenters[j][1]), obstacleRadii[j], color = 'black')
                ax.add_patch(thisCircle)
            plt.savefig("caseTwo/caseTwo" + str(i + 1) + ".png")
        print("FINISHED ITERATION: ", i)

    plotAndSaveNodeTrajectories(allPositions, "caseTwo/caseTwo_trajectory.png", "Sin Wave")
    plotAndSaveNodeVelocities(allNodeVelocities, "caseTwo/caseTWo_individual_velocity.png", "caseTwo/caseTwo_all_velcoity.png", "Sin Wave")
    # plotAndSaveConnectivity(allPositions, "caseTwo/caseTwo_connectivity.png", "Sin Wave")
    plotCenterOfMassAndTarget(gammaX, gammaY, allPositions, "caseTwo/center_of_mass.png", "Sin Wave")


def updateAlgo3(nodePositions, nodeVelocities, gammaPos, gammaVelocity, obstacleCenters, obstacleRadii):

    newAcclerations = np.zeros((DIMENSIONS, NUM_NODES))
    newVelocities = np.zeros((DIMENSIONS, NUM_NODES))
    newPositions = np.zeros((DIMENSIONS, NUM_NODES))
    # we must update the acceleration for every node
    for i in range(0, NUM_NODES):

        sumOfFirstPart = (0, 0)
        sumOfSecondPart = (0, 0)
        q_i = (nodePositions[0][i], nodePositions[1][i])
        p_i = (nodeVelocities[0][i], nodeVelocities[1][i])
        for j in range(0, NUM_NODES):
            if i == j:
                continue

            thisPairNorm = euclidean_norm(tuple_diff((nodePositions[0][i], nodePositions[1][i]), (nodePositions[0][j], nodePositions[1][j])))
            if thisPairNorm > R:
                continue

            q_j = (nodePositions[0][j], nodePositions[1][j])
            p_j = (nodeVelocities[0][j], nodeVelocities[1][j])
            # this is a neighbor node, need to calculate its effect for the new acceleration
            # first part
            this_sigma_norm = sigma_norm(tuple_diff((q_j[0], q_j[1]), (q_i[0], q_i[1])))
            this_phi_alpha = phi_alpha(this_sigma_norm)
            this_n_i_j = n_i_j(q_j, q_i)
            this_part = tuple_by_scalar(this_n_i_j, this_phi_alpha)
            sumOfFirstPart = tuple_sum(sumOfFirstPart, this_part)
            # seoncd part
            this_a_i_j = a_i_j(q_j, q_i)
            velocity_diff = tuple_diff(p_j, p_i)
            secondPart = tuple_by_scalar(velocity_diff, this_a_i_j)
            sumOfSecondPart = tuple_sum(sumOfSecondPart, secondPart)

        sumOfFirstPart = tuple_by_scalar(sumOfFirstPart, C_1_ALPHA)
        sumOfSecondPart = tuple_by_scalar(sumOfSecondPart, C_2_ALPHA)

        # gamma pos
        staticTarget = tuple_diff(q_i, gammaPos)
        staticTarget = tuple_by_scalar(staticTarget, C_MT_1)
        newAccel = tuple_sum(sumOfFirstPart, sumOfSecondPart)
        newAccel = tuple_diff(newAccel, staticTarget)

        # gamma velocity
        gammaVel = tuple_diff(p_i, gammaVelocity)
        gammaVel = tuple_by_scalar(gammaVel, C_MT_2)
        newAccel = tuple_diff(newAccel, gammaVel)

        # obstacle avoidance
        for j in range(0, len(obstacleCenters)):

            y_k = (obstacleCenters[j][0], obstacleCenters[j][1])
            thisObstacleRadius = obstacleRadii[j]
            thisMu = mu(thisObstacleRadius, q_i, y_k)
            thisQIK = q_hat_i_k(thisMu, q_i, y_k)

            # determine if we are close enough to the obstacle to have an effect from it
            if(euclidean_norm(tuple_diff(thisQIK, q_i)) < R_PRIME):
                # within the interaction range of this obstacle!
                firstPart = tuple_by_scalar(n_hat_i_k(thisQIK, q_i), phi_beta(sigma_norm(tuple_diff(thisQIK, q_i))))
                firstPart = tuple_by_scalar(firstPart, C_1_BETA)
                thisBIK = b_i_k(thisQIK, q_i)
                secondPart = tuple_diff(p_hat_i_k(thisMu, P(a_k(q_i, y_k)), p_i), p_i)
                secondPart = tuple_by_scalar(secondPart, thisBIK)
                secondPart = tuple_by_scalar(secondPart, C_2_BETA)
                accelFromObstacle = tuple_sum(firstPart, secondPart)
                newAccel = tuple_sum(newAccel, accelFromObstacle)

                
        newAcclerations[0][i] = newAccel[0] # x
        newAcclerations[1][i] = newAccel[1] # y
        newVelocities[0][i] = nodeVelocities[0][i] + (newAccel[0] * DELTA_T) # x
        newVelocities[1][i] = nodeVelocities[1][i] + (newAccel[1] * DELTA_T) # y
        newPositions[0][i] = nodePositions[0][i] + (nodeVelocities[0][i] * DELTA_T) + ((1/2) * newAccel[0] * (DELTA_T * DELTA_T))
        newPositions[1][i] = nodePositions[1][i] + (nodeVelocities[1][i] * DELTA_T) + ((1/2) * newAccel[1] * (DELTA_T * DELTA_T))

    return newPositions, newVelocities, newAcclerations

def euclidean_norm(vector):
    """
    Input: Vector (x, y)
    Output: Euclidean Norm: Scalar
    """
    squaredSum = 0
    for component in vector:
        squaredSum += (component * component)
    return sqrt(squaredSum)

def sigma_norm(vector):
    """
    Input: Vector (x, y)
    Output: Sigma Norm: Scalar
    """
    euclidean = euclidean_norm(vector)
    squared_euclidean = euclidean * euclidean
    return (1 / EPSILON) * (sqrt(1 + (EPSILON * squared_euclidean)) - 1)

def n_i_j(vector1, vector2):
    """
    Input: Two vectors (x, y), (x, y)
    Output: One vector (x, y)
    """
    difference = tuple_diff(vector1, vector2)
    euclidean = euclidean_norm(difference)
    squared_euclidean = euclidean * euclidean

    return tuple_div_by_scalar(difference, (sqrt(1 + (EPSILON * squared_euclidean))))

def tuple_diff(vector1, vector2):
    """
    Input: Two vectors (x, y), (x, y)
    Output: One vector (x, y), result of vector1 - vector2
    """
    return tuple(np.subtract(vector1, vector2))

def tuple_sum(vector1, vector2):
    """
    Input: Two vectors (x, y), (x, y)
    Output: One vector (x, y), result of vector1 + vector 2
    """
    return tuple(np.add(vector1, vector2))

def tuple_by_scalar(vector, scalar):
    """
    Input: One vector (x, y), One scalar, z
    Output: One vector (x, y), result of each element in vector being multiplied by scalar
    """
    return tuple([scalar * component for component in vector])

def tuple_div_by_scalar(vector, scalar):
    """
    Input: One vector (x, y), One scalar, z
    Output: One vector (x, y), result of each element in vector being divided by scalar
    """
    return tuple([component / scalar for  component in vector])

def rho_h(z):
    """
    Input: One scalar value, z
    Output: One scalar value, z
    """
    if z >= 0 and z < H:
        return 1
    elif z >= H and z <= 1:
        return ((1 / 2) * (1 + cos(pi * ((z - H)/(1 - H)))))
    else:
        return 0
    
R_ALPHA = sigma_norm((R,))
D_ALPHA = sigma_norm((D,))
R_BETA = sigma_norm((R_PRIME,))
D_BETA = sigma_norm((D_PRIME,))

def a_i_j(vector1, vector2):
    """
    Input: Two vectors (x, y), (x, y)
    Output: One scalar value, z
    """
    difference = tuple_diff(vector1, vector2)
    sigma_of_diff = sigma_norm(difference)
    rho = rho_h(sigma_of_diff / R_ALPHA)
    return rho

def phi_alpha(z):
    """
    Input: One scalar value, z
    Output: One scalar value, z
    """
    this_rho_h = rho_h(z / R_ALPHA)
    this_phi = phi(z - D_ALPHA)
    return this_rho_h * this_phi

def phi(z):
    """
    Input: One scalar value, z
    Output: One scalar value, z
    """
    bracket_portion = ((A + B) * sigma_one(z + C)) + (A - B)
    return ((1 / 2) * bracket_portion)

def sigma_one(z):
    """
    Input: One scalar value, z
    Output: One scalar value, z
    """
    return (z / (sqrt(1 + (z * z))))

def mu(R_k, q_i, y_k):
    """
    Input: Scalar value, R_k: Radus of obstacle k
           Vector, q_i: position of node i
           Vector, y_k: Position of center of obstacle k
    Output: scalar, mu
    """
    bottomPart = euclidean_norm(tuple_diff(q_i, y_k))
    return R_k / bottomPart

def a_k(q_i, y_k):
    """
    Input: Vector, q_i: Position of node i
           Vector, y_k: Position of center of obstacle k
    Output: Vector, a_k
    """
    topPart = tuple_diff(q_i, y_k)
    bottomPart = euclidean_norm(tuple_diff(q_i, y_k))
    return tuple_div_by_scalar(topPart, bottomPart)

def q_hat_i_k(mu, q_i, y_k):
    """
    Input: Scalar, mu
           Vector, q_i: Position of node i
           Vector, y_k: Position of center of obstacle k
    Output: Vector, q_hat_i_k
    """
    firstPart = tuple_by_scalar(q_i, mu)
    secondPart = tuple_by_scalar(y_k, (1 - mu))
    return tuple_sum(firstPart, secondPart)

def P(a_k):
    """
    Input: a_k: Vector, output from a_k function
    Output: P, a list of tuples representing a matrix
    """
    P = []
    for i in a_k:
        thisRow = []
        for j in a_k:
            thisRow.append(i * j)
        P.append(thisRow)
    
    for i in range(0, len(P)):
        for j in range(0, len(P[i])):
            if i == j:
                P[i][j] = 1 - P[i][j]
            else:
                P[i][j] = 0 - P[i][j]
    return P

def p_hat_i_k(mu, P, p_i):
    """
    Input: Scalar, mu
           Matrix, P,
           Vector, p_i
    Output: Vector
    """
    for i in range(0, len(P)):
        for j in range(0, len(P[i])):
            P[i][j] = mu * P[i][j]

    # use numpy for matrix multiplication
    P = np.array(P)
    p_i = np.array(p_i)
    return tuple(np.dot(P, p_i))
    
def b_i_k(q_i_k, q_i):
    """
    Input:
           vector qhat_i_k,
           vector q_i
    Output: scalar
    """
    sig_norm = sigma_norm(tuple_diff(q_i_k, q_i))
    rho = rho_h(sig_norm / D_BETA)
    return rho

def phi_beta(z):
    """
    Input: scalar z
    Output: scalar
    """
    rho_part = rho_h(z / D_BETA)
    other_part = (sigma_one(z - D_BETA) - 1)
    return rho_part * other_part

def n_hat_i_k(q_hat_i_k, q_i):
    """
    Two vectors
    One vector
    """
    topPart = tuple_diff(q_hat_i_k, q_i)
    euclid = euclidean_norm(tuple_diff(q_hat_i_k, q_i))
    euclid = euclid * euclid
    bototmPart = sqrt(1 + EPSILON * euclid)
    return tuple_div_by_scalar(topPart, bototmPart)
    
def plotNodesAndSave(nodePositions, fileName):

    plt.clf()
    for i in range(0, NUM_NODES):
        for j in range(0, NUM_NODES):

            # nodes should not be used with themselves
            if i == j:
                continue

            thisPairNorm = euclidean_norm(tuple_diff((nodePositions[0][i], nodePositions[1][i]), (nodePositions[0][j], nodePositions[1][j])))
            if thisPairNorm < R:
                # add line to plot for part 4
                plt.plot([nodePositions[0][i], nodePositions[0][j]],[nodePositions[1][i], nodePositions[1][j]], color = "blue", linewidth = "0.5")
    plt.scatter(nodePositions[0], nodePositions[1], marker = ">", color = "magenta")
    plt.gcf().gca().set_aspect("equal")
    plt.savefig(fileName)

def plotNodesAndObstaclesAndSave(nodePositions, fileName, obstacleRadii, obstaclePositions):

    plt.clf()
    for i in range(0, NUM_NODES):
        for j in range(0, NUM_NODES):

            # nodes should not be used with themselves
            if i == j:
                continue

            thisPairNorm = euclidean_norm(tuple_diff((nodePositions[0][i], nodePositions[1][i]), (nodePositions[0][j], nodePositions[1][j])))
            if thisPairNorm < R:
                # add line to plot for part 4
                plt.plot([nodePositions[0][i], nodePositions[0][j]],[nodePositions[1][i], nodePositions[1][j]], color = "blue", linewidth = "0.5")
    plt.scatter(nodePositions[0], nodePositions[1], marker = ">", color = "magenta")

    ax = plt.gcf().gca()
    for i in range(0, len(obstacleRadii)):
        thisCircle = plt.Circle((obstaclePositions[i][0], obstaclePositions[i][1]), obstacleRadii[i], color = 'black')
        ax.add_patch(thisCircle)

    plt.gcf().gca().set_aspect("equal")
    plt.savefig(fileName)

def plotNodes(nodePositions):

    # plt.clf()
    for i in range(0, NUM_NODES):
        for j in range(0, NUM_NODES):

            # nodes should not be used with themselves
            if i == j:
                continue

            thisPairNorm = euclidean_norm(tuple_diff((nodePositions[0][i], nodePositions[1][i]), (nodePositions[0][j], nodePositions[1][j])))
            if thisPairNorm < R:
                # add line to plot for part 4
                plt.plot([nodePositions[0][i], nodePositions[0][j]],[nodePositions[1][i], nodePositions[1][j]], color = "blue", linewidth = "0.5")
    plt.scatter(nodePositions[0], nodePositions[1], marker = ">", color = "magenta")
    plt.gcf().gca().set_aspect("equal")

def plotAndSaveNodeTrajectories(allPositions, fileName, sim):
    plt.clf()
    # shape[0] is x, y
    # shape[1] is each node
    # shape[2] is each time step
    for i in range(0, allPositions.shape[1]):
        thisNodeX = allPositions[0, i, :]
        thisNodeY = allPositions[1, i, :]
        plt.plot(thisNodeX, thisNodeY, color = "black")
        plt.title("Node Trajectories - " + sim)
    plt.scatter(allPositions[0, :, allPositions.shape[2] - 1], allPositions[1, :, allPositions.shape[2] - 1], marker=">", color = "magenta")
    plt.gcf().gca().set_aspect("equal")
    plt.savefig(fileName)

def plotAndSaveNodeVelocities(allVelocities, fileName, fileName2, sim):
    plt.clf()
    for i in range(0, allVelocities.shape[1]):
        thisNodeX = allVelocities[0, i, 1:]
        thisNodeY = allVelocities[1, i, 1:]
        mag = np.sqrt(np.add(np.power(thisNodeX, 2), np.power(thisNodeY, 2)))
        plt.plot(mag)
    plt.title("Node Velocities - " + sim)
    plt.savefig(fileName)

    plt.clf()
    averageVelcocities = []
    for i in range(0, allVelocities.shape[2]):
        thisMag = 0
        for j in range(0, allVelocities.shape[1]):
            thisMag += sqrt((allVelocities[0, j, i] * allVelocities[0, j, i]) + (allVelocities[1, j, i] * allVelocities[1, j, i]))
        thisMag /= allVelocities.shape[1]
        averageVelcocities.append(thisMag)

    plt.plot(averageVelcocities)
    plt.title("Average Velocity of All Nodes - " + sim)
    plt.savefig(fileName2)

def plotAndSaveConnectivity(allPositions, fileName, sim):
    plt.clf()
    # loop for every time point
    connectivity = []
    for i in range(0, allPositions.shape[2]):
        emptyMatrix = np.zeros((allPositions.shape[1], allPositions.shape[1]))
        # loop through every node to make the matrix
        for j in range(0, allPositions.shape[1]):
            for k in range(0, allPositions.shape[1]):
                if j == k:
                    continue
                thisPairNorm = euclidean_norm(tuple_diff((allPositions[0][j][i], allPositions[1][j][i]), (allPositions[0][k][i], allPositions[1][k][i])))
                if thisPairNorm < R:
                    emptyMatrix[j][k] = 1

        rank = np.linalg.matrix_rank(emptyMatrix)
        connectivity.append((1 / NUM_NODES) * rank)
    plt.plot(connectivity)
    plt.title("Connectivity - " + sim)
    plt.savefig(fileName)

def plotCenterOfMassAndTarget(targetX, targetY, nodePositions, filename, sim):
    plt.clf()
    x_vels = nodePositions[0, :, :]
    x_sums = np.sum(x_vels, axis = 0)
    x_sums = np.divide(x_sums, nodePositions.shape[1])
    y_vels = nodePositions[1, :, :]
    y_sums = np.sum(y_vels, axis = 0)
    y_sums = np.divide(y_sums, nodePositions.shape[1])
    plt.plot(x_sums, y_sums, color = "black", label = "Center of Mass")
    plt.plot(targetX, targetY, color = "red", label = "Gamma Agent")
    plt.legend(loc = "lower left")
    plt.title("Center of Mass vs. Gamma Agent - " + sim)
    plt.savefig(filename)


if __name__ == "__main__":
    main()
