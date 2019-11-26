#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pickle

with open("mapasgraph2.pickle", "rb") as fp:   #Unpickling
    AA = pickle.load(fp)

#######################
# CUIDADO: script usa a primeira representação do grafo, que tem algumas diferenças para a segunda, que é a que é suposto usar
#######################

U = AA[0]
L = AA[1]

print("113 localizações + 1 lista (na posição 0) vazia. As representações são equivalentes.\n")

print("Representação 1: cada entrada i da lista é uma lista com 4 listas, em que a primeira (0) indica as localizações acessíveis diretamente a partir de i de táxi, a segunda (1) de autocarro, a terceira (2) de metro e a quarta (3) de barco.\n")
print(str(U))
print()
print()

print("Representação 2: cada entrada i da lista é uma lista de listas [x, y] de possíveis transições da localização i para a localização y usando o meio de transporte x (0 = táxi, 1 = autocarro, 2 = metro, 3 = barco).\n")
print(str(L))
print()
print()

print("Representação 3: explícita\n")

print("Locais acessíveis a partir de:\n")
for i, el in enumerate(U):
    if i == 0:
        continue
    print("Local " + str(i) + ":\n")
    print("Taxi (0): " + str(el[0]))
    print("Autocarro (1): " + str(el[1]))
    print("Metro (2): " + str(el[2]))
    print("Barco (3): " + str(el[3]))
    print()