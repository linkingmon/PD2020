import matplotlib.pyplot as plt
import sys


def plot_blocks(L):
    # plot my outline
    # plot real outline
    # [max_x, max_y] = [int(t) for t in L[5].strip('\n').split(' ')]
    # plt.plot((0, 0), (0, max_y,), 'g-')
    # plt.plot((0, max_x), (max_y, max_y,), 'g-')
    # plt.plot((max_x, max_x), (max_y, 0,), 'g-')
    # plt.plot((max_x, 0), (0, 0,), 'g-')
    # plot blocks
    cnt = 6
    while cnt < len(L):
        sub_list = L[cnt].strip('\n').split(' ')
        word_list = []
        for word in sub_list:
            if word != '':
                word_list.append(word)
        if(word_list[0] == 'solution'):
            # if(word_list[0] == 'net'):
            break
        else:
            name = word_list[0]
            x1 = int(word_list[1])
            y1 = int(word_list[2])
            x2 = int(word_list[3])
            y2 = int(word_list[4])
            print(x1, y1, x2, y2)
            plt.plot((x1, x1), (y1, y2,), 'r-')
            plt.plot((x1, x2), (y2, y2,), 'r-')
            plt.plot((x2, x2), (y2, y1,), 'r-')
            plt.plot((x2, x1), (y1, y1,), 'r-')
            plt.text((x1+x2)/2, (y1+y2)/2, name, fontsize=7)
        cnt += 1


def plot_net(cnt, idx):
    sub_list = L[cnt+idx].strip('\n').split(' ')
    print(sub_list)
    word_list = []
    for word in sub_list:
        if word != '':
            word_list.append(word)
    wire_len = word_list[2]
    print("HPWL of net %d is %s" % (idx, wire_len))
    for idx in range(3, len(word_list), 2):
        print(float(word_list[idx]), float(word_list[idx+1]))
        plt.plot(float(word_list[idx]), float(word_list[idx+1]), marker='*', color='r')


def plot_hist(cnt, cur_area, cur_wire):
    alhpa = 0.5
    areas = []
    wires = []
    while cnt < len(L):
        sub_list = L[cnt].strip('\n').split(' ')
        word_list = []
        for word in sub_list:
            if word != '':
                word_list.append(word)
        areas.append(int(word_list[1]))
        wires.append(float(word_list[2]))
        cnt += 1
    area_norm = sum(areas) / len(areas)
    wire_norm = sum(wires) / len(wires)
    costs = []
    for i in range(len(areas)):
        costs.append(areas[i]*alhpa/area_norm+(1-alhpa)*wires[i]/wire_norm)

    plt.clf()
    plt.subplot(3, 1, 1)
    plt.hist(areas, bins=1000)
    plt.plot(cur_area, 0, 'ro')
    plt.title('Area')

    plt.subplot(3, 1, 2)
    plt.hist(wires, bins=1000)
    plt.plot(cur_wire, 0, 'ro')
    plt.title('Wire')

    alpha = 0.5
    plt.subplot(3, 1, 3)
    plt.hist(costs, bins=1000)
    plt.plot(cur_area*alhpa/area_norm+(1-alpha)*cur_wire/wire_norm, 0, 'ro')
    plt.title('Cost')
    plt.show()


def plot_outline(L):
    [_, max_x, max_y] = [t for t in L[0].strip('\n').split(' ')]
    max_x = int(max_x)
    max_y = int(max_y)
    plt.plot((0, 0), (0, max_y,), 'g-')
    plt.plot((0, max_x), (max_y, max_y,), 'g-')
    plt.plot((max_x, max_x), (max_y, 0,), 'g-')
    plt.plot((max_x, 0), (0, 0,), 'g-')
    plt.text(max_x, (max_y)/3, 'outline', fontsize=7)


def plot_bound_box(L):
    [max_x, max_y] = [int(t) for t in L[3].strip('\n').split(' ')]
    plt.plot((0, 0), (0, max_y,), 'b-')
    plt.plot((0, max_x), (max_y, max_y,), 'b-')
    plt.plot((max_x, max_x), (max_y, 0,), 'b-')
    plt.plot((max_x, 0), (0, 0,), 'b-')
    plt.text(max_x, (max_y*2)/3, 'bound box', fontsize=7)


if __name__ == "__main__":
    filename = sys.argv[1]
    filename2 = sys.argv[2]
    L = open(filename).readlines()
    # cur_wire = float(L[1].strip('/n'))
    # cur_area = int(L[2].strip('/n'))
    plot_bound_box(L)
    L2 = open(filename2).readlines()
    plot_outline(L2)
    plot_blocks(L)
    plt.title('Resulting Floorplan')
    plt.show()
