import pickle
import copy
import matplotlib.pyplot as plt
import time
from ruagomesfreiregamesol import SearchProblem

with open("coords.pickle", "rb") as fp:   # Unpickling
    coords = pickle.load(fp)

with open("mapasgraph.pickle", "rb") as fp:  # Unpickling
    AA = pickle.load(fp)
U = AA[1]


def plotpath(P, coords):
    img = plt.imread('maps.png')
    plt.imshow(img)
    colors = ['r.-', 'g+-', 'b^-']
    I = P[0][1]
    for agind in range(len(P[0][1])):
        st = I[agind]-1
        for tt in P:
            nst = tt[1][agind]-1
            plt.plot([coords[st][0], coords[nst][0]], [
                     coords[st][1], coords[nst][1]], colors[agind])
            st = nst
    plt.axis('off')
    fig = plt.gcf()
    fig.set_size_inches(1.*18.5, 1.*10.5)
    #fig.savefig('test2png.png', dpi=100)
    plt.show()


def validatepath(oP, I, U, tickets=[25, 25, 25]):
    if not oP:
        return False
    P = copy.deepcopy(oP)
    del P[0]
    for tt in P:
        for agind, ag in enumerate(tt[1]):
            # print(ag)
            st = I[agind]
            if tickets[tt[0][agind]] == 0:
                print('no more tickets')
                return False
            else:
                tickets[tt[0][agind]] -= 1

                if [tt[0][agind], ag] in U[st]:
                    I[agind] = ag
                    # pass
                else:
                    print('invalid action')
                    return False
    return True

for i in range(1, 114):
    print("\n(4 val) Exercise 2 - One agent, Limits")
    print("Init [" + str(i) + "] Goal [56]")
    SP = SearchProblem(goal=[56], model=U, auxheur=coords)
    lst = SP.getHeur([5, 5, 2])
    tinit = time.process_time()
    I = [i]
    nn = SP.search(I, limitexp=2000, tickets=[5, 5, 2])
    tend = time.process_time()
    print("%.1fms" % ((tend-tinit)*1000))
    if validatepath(nn, I, U, tickets=[5, 5, 2]):
        print("path")
        print(nn)
        if len(nn)-1 < lst[i]:
            print()
            print()
            print("----------------------------------------")
            print("INCONSISTENCY DETECTED IN TEST " + str(i))
            print("----------------------------------------")
            print()
            print()
    else:
        print("invalid path")
