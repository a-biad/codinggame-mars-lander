import os

for i in range(1,5 + 1):
    os.system("./a.out < input/input{}.txt | tee result/resultB{}.txt".format(i,i))

