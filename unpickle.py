import pickle

pickle_off = open("coords.pickle","rb")
emp = pickle.load(pickle_off)
print("coords: (" + str(len(emp)) + " posições)")
print(emp)

pickle_off2 = open("mapasgraph.pickle","rb")
emp2 = pickle.load(pickle_off2)
print("mapasgraph:")
for i in range(len(emp2)):
    print("Mapa " + str(i) + ": (" + str(len(emp2[i])-1) + " posições)")
    print(emp2[i])