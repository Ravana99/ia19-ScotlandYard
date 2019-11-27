import random
import numpy

# LearningAgent to implement
# no knowledeg about the environment can be used
# the code should work even with another environment
class LearningAgent:

	# init
	# nS maximum number of states
	# nA maximum number of action per state
	def __init__(self,nS,nA):

		# define this function
		self.nS = nS
		self.nA = nA

		self.qtable = [[0]*nA for i in range(nS)]
		self.nvisits = [[0]*nA for i in range(nS)]
		self.gamma = 1
		self.nactions = [-1]*nS
		  
	
	# Select one action, used when learning  
	# st - is the current state        
	# aa - is the set of possible actions
	# for a given state they are always given in the same order
	# returns the index to the action in aa
	def selectactiontolearn(self,st,aa):
		# define this function
		# print("select one action to learn better")
		if self.nactions[st] == -1:
			self.nactions[st] = len(aa)
		return random.randint(0, self.nactions[st]-1)

	# Select one action, used when evaluating
	# st - is the current state        
	# aa - is the set of possible actions
	# for a given state they are always given in the same order
	# returns
	# imax - the index to the action in aa
	def selectactiontoexecute(self,st,aa):
		# define this function
		if self.nactions[st] == -1:
			self.nactions[st] = len(aa)
		amax = self.qtable[st][0]
		imax = 0
		for i in range(1, self.nactions[st]):
			if amax < self.qtable[st][i]:
				amax = self.qtable[st][i]
				imax = i

		# print("select one action to see if I learned")
		print(st)
		return imax


	# this function is called after every action
	# ost - original state
	# nst - next state
	# a - the index to the action taken
	# r - reward obtained
	def learn(self,ost,nst,a,r):
		# define this function
		#print("learn something from this data")
		#if self.nvisits[ost][a] == 0:
			#self.qtable[ost][a] = r
		self.nvisits[ost][a] += 1
		alphan = 1/(self.nvisits[ost][a]+1)
		amax = self.qtable[nst][0]
		if self.nactions[nst] == -1:
			amax = 0
		else:	
			for i in range(1, self.nactions[nst]):
				if amax < self.qtable[nst][i]:
					amax = self.qtable[nst][i]

		self.qtable[ost][a] = (1-alphan)*self.qtable[ost][a] + alphan*(r+self.gamma*amax)

		return