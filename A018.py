# Group 18 - Joao Fonseca 89476, Tomas Lopes 89552

import random

# LearningAgent to implement
# no knowledge about the environment can be used
# the code should work even with another environment
class LearningAgent:

	# init
	# nS maximum number of states
	# nA maximum number of action per state
	def __init__(self,nS,nA):

		self.nS = nS
		self.nA = nA

		self.qtable = [[0]*nA for i in range(nS)]
		self.nvisits = [[0]*nA for i in range(nS)]
		self.gamma = 0.999
		self.nactions = [-1]*nS
		self.epsilon = 0.7

		  
	
	# Select one action, used when learning  
	# st - is the current state  
	# aa - is the set of possible actions
	# for a given state they are always given in the same order
	# returns the index to the action in aa
	def selectactiontolearn(self,st,aa):
		if self.nactions[st] == -1:
			self.nactions[st] = len(aa)

		self.epsilon -= 1/(self.nS*self.nA*self.nA)

		if random.random()<self.epsilon:
			return random.randint(0, self.nactions[st]-1)	
		else:
			return self.selectactiontoexecute(st,aa)
			

	# Select one action, used when evaluating
	# st - is the current state        
	# aa - is the set of possible actions
	# for a given state they are always given in the same order
	# returns the index to the action in aa
	def selectactiontoexecute(self,st,aa):
		if self.nactions[st] == -1:
			self.nactions[st] = len(aa)
		maxlist = []
		amax = self.qtable[st][0]
		for i in range(1, self.nactions[st]):
			if amax < self.qtable[st][i]:
				amax = self.qtable[st][i]
		for i in range(0, self.nactions[st]):
			if amax == self.qtable[st][i]:
				maxlist.append(i)

		return maxlist[random.randint(0, len(maxlist)-1)]


	# this function is called after every action
	# ost - original state
	# nst - next state
	# a - the index to the action taken
	# r - reward obtained
	def learn(self,ost,nst,a,r):

		for i in range(self.nA):
			if self.nvisits[ost][i] == 0:
				self.qtable[ost][i] = r

		self.nvisits[ost][a] += 1

		alphan = 1/(self.nvisits[ost][a]+1)

		amax = self.qtable[nst][0]
		if self.nactions[nst] == -1:
			amax = 0

		for i in range(1, self.nactions[nst]):
			if amax < self.qtable[nst][i]:
				amax = self.qtable[nst][i]

		self.qtable[ost][a] = (1-alphan)*self.qtable[ost][a] + alphan*(r+self.gamma*amax)

		return