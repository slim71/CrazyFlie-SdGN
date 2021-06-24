import time


def read_point(desc):
    print(desc.readline().split(" ")[0] == "%")
    line = desc.readline()
    line = line.split(" ")
    print("line is ", line)
    point = []
    index = 0
    for element in line:
        element = element.strip()
        print("element is ", element)
        point.append(element)
        index += 1
        if index >= 3:
            break
    return point


fr = open("../wand_data/crazyfun__20210611_165600.txt", mode='a')

with open("../wand_data/crazyfun__20210611_165600.txt", mode='r') as ff:
    ll = read_point(ff)
    print("ll ", ll)
    time.sleep(5)
    l2 = read_point(ff)
    print("l2 ", l2)
