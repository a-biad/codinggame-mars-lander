import os

for i in range(1,5 + 1):
    os.system("./a.out < input/input{}.txt | tee result/resultE{}.txt".format(i,i))

