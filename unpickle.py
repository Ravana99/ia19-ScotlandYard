import pickle


pickle_off = open("mapasgraph2.pickle","rb")
emp = pickle.load(pickle_off)
print("mapasgraph:")
print(emp)

#for i in range(len(emp2)):
 #   print("Mapa " + str(i) + ": (" + str(len(emp2[i])-1) + " posições)")
  #  print(emp2[i])
