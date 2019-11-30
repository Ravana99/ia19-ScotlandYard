import pickle
import random
import matplotlib.pyplot as plt
from ruagomesfreiregame2sol import *

def runagent(A, T, R, I = 1, learningphase=True, nlearn = 1000, ntest = 100):

        J = 0 #J -> sum of all rewards received
        if learningphase:
                n = nlearn #n -> number of tests
        else:
                n = ntest #n -> number of learns
                
        st = I #state = I (I -> initial state)
        for ii in range(1,n): #executes n tests/learns
                aa = T[st][0] #possible next states after the current state st
                if learningphase:
                        a = A.selectactiontolearn(st,aa) #a -> action to learn
                else:
                        a = A.selectactiontoexecute(st,aa) #a -> action to execute
                try:
                        nst = T[st][0][a] # nst -> next state using the selected action a
                except:
                        print(st,a)
                r = R[st] #r -> reward associated with the current state st
                J += r #adds current reward to the sum of all rewards received
                #print(st,nst,a,r)

                if learningphase:
                        A.learn(st,nst,a,r)
                else:
                        #print(st,nst,a,r)
                        pass
                
                st = nst #state = next state

                if not ii%15: #after 15 iterations return to the intial state
                        st = I
        return J/n #returns average of all rewards received by number of tests/learns
        

# due to the randomness in the learning process, we will run everything NREP times
# the final grades are based on the average on all of them

NIT = 5
NREP = 100 #number of times the whole process will be executed
hits0 = 0
hits1 = 0
hits2 = 0
hits3 = 0
hitsu0 = 0
hitsu1 = 0
hitsu2 = 0
hitsu3 = 0

with open("mapasgraph2.pickle", "rb") as fp:
        AA = pickle.load(fp)

for i in range(NREP):

        val = [0,0,0,0]

        for nrep in range(NIT):

                A = LearningAgent(114,15)

                T = AA[0]
                R = [-1]*114
                R[7] = 1
                R[1] = 0
                R[2] = 0
                R[3] = 0
                R[4] = 0

                runagent(A, T, R, I = 1, learningphase=True, nlearn = 500)
                Jn = runagent(A, T, R, I = 1, learningphase=False, ntest = 10)
                val[0] += Jn

                if Jn >= 0.3:
                    hitsu0 += 1


                runagent(A, T, R, I = 1, learningphase=True, nlearn = 10000)
                Jn = runagent(A, T, R, I = 1, learningphase=False, ntest = 10)
                val[1] += Jn

                if Jn >= 0.3:
                    hitsu1 += 1

        for nrep in range(NIT):
                
                A = LearningAgent(114,15)

                T = AA[0]
                R = [-1]*114
                R[10] = 1

                runagent(A, T, R, I = 1, learningphase=True, nlearn = 1000)
                Jn = runagent(A, T, R, I = 1, learningphase=False, ntest = 10)
                val[2] += Jn

                if Jn >= -0.85:
                    hitsu2 += 1
                
                runagent(A, T, R, I = 1, learningphase=True, nlearn = 10000)
                Jn = runagent(A, T, R, I = 1, learningphase=False, ntest = 10)
                val[3] += Jn

                if Jn >= -0.6:
                    hitsu3 += 1


        val = [val[i]/NIT for i in range(4)]

        if val[0]>=0.3:
            hits0 += 1
        #else:
            #print("MISS CONJUNTO NO EXEMPLO 1 TESTE 1")
        if val[1]>=0.3:
            hits1 += 1
        #else:
            #print("MISS CONJUNTO NO EXEMPLO 1 TESTE 2")
        if val[2]>=-0.85:
            hits2 += 1
        #else:
            #print("MISS CONJUNTO NO EXEMPLO 2 TESTE 1")
        if val[3]>=-0.6:
            hits3 += 1
        #else:
            #print("MISS CONJUNTO NO EXEMPLO 2 TESTE 2")


print("RESULTADOS ÚNICOS:")
print("Hits exemplo 1 teste 1: " +  str(hitsu0) + " em " + str(NREP*NIT))
print("% Hits exemplo 1 teste 1: " + str(hitsu0*100/(NREP*NIT)) + "%")
print("Hits exemplo 1 teste 2: " +  str(hitsu1) + " em " + str(NREP*NIT))
print("% Hits exemplo 1 teste 2: " + str(hitsu1*100/(NREP*NIT)) + "%")
print("Hits exemplo 2 teste 1: " +  str(hitsu2) + " em " + str(NREP*NIT))
print("% Hits exemplo 2 teste 1: " + str(hitsu2*100/(NREP*NIT)) + "%")
print("Hits exemplo 2 teste 2: " +  str(hitsu3) + " em " + str(NREP*NIT))
print("% Hits exemplo 2 teste 2: " + str(hitsu3*100/(NREP*NIT)) + "%")

print("RESULTADOS CONJUNTOS:")
print("Hits exemplo 1 teste 1: " +  str(hits0) + " em " + str(NREP))
print("% Hits exemplo 1 teste 1: " + str(hits0*100/NREP) + "%")
print("Hits exemplo 1 teste 2: " +  str(hits1) + " em " + str(NREP))
print("% Hits exemplo 1 teste 2: " + str(hits1*100/NREP) + "%")
print("Hits exemplo 2 teste 1: " +  str(hits2) + " em " + str(NREP))
print("% Hits exemplo 2 teste 1: " + str(hits2*100/NREP))
print("Hits exemplo 2 teste 2: " +  str(hits3) + " em " + str(NREP))
print("% Hits exemplo 2 teste 2: " + str(hits3*100/NREP))
