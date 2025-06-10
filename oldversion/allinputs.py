import os

for i in range(1,5 + 1):
    os.system("./main2 < input/input{}.txt | tee result/resultH{}.txt".format(i,i))

