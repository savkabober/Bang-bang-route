from time import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

# Задаем известные величины
r = np.array([90, 40])  # Пример вектора r
Am = 1 # Пример числа Am
Va = np.array([-10, 10])  # Пример вектора Va
Vb = np.array([5, -12])  # Пример вектора Vb
Vmax = 13
delay = 0.001
cos_tolerance = 1e-8
ang_tolerance = 1e-8
r_tolerance = 1e-8
v_tolerance = 1e-8



# Определяем уравнение
def short_dist(Vm):
    Vm = np.array(Vm)
    return 2 * Am * r - (Vm + Va) * np.linalg.norm(Vm - Va) - (Vm + Vb) * np.linalg.norm(Vm - Vb)

def long_dist(ang):
    Vm = np.array([Vmax * np.cos(ang[0]), Vmax * np.sin(ang[0])])
    lhs = 2 * Am * r - (Vm + Va) * np.linalg.norm(Vm - Va) - (Vm + Vb) * np.linalg.norm(Vm - Vb)
    return [(lhs[0] * Vm[0] + lhs[1] * Vm[1]) / np.linalg.norm(lhs) / np.linalg.norm(Vm) - 1]

def plot_way(points):

    if not points or len(points) < 2:
        print("< 2 точек")
        return
    
    xs, ys = [], []

    for i in points:
        xs.append(i[0])
        ys.append(i[1])
    # Создаем график
    plt.plot(xs, ys, marker=".", label="Траектория")

    # Подписываем первую и последнюю точки
    plt.text(xs[0], ys[0], f"{points[0]}", fontsize=10, color="green", ha='right')
    plt.text(xs[-1], ys[-1], f"{points[-1]}", fontsize=10, color="red", ha='left')

    plt.grid(True, linestyle="--", alpha=0.7)
    plt.legend()

    plt.show()

if np.linalg.norm(r) == 0:
    print("Мы уже в нужной точке")
else:
    # Vm_initial = r / np.linalg.norm(r) * Vmax

    # # Решение уравнения
    # time_initial = time()
    # short_solution = least_squares(short_dist, Vm_initial, bounds=(-Vmax, Vmax))
    # time_final = time()

    # print("Время поиска решения:", (time_final - time_initial) * 1000)

    # Vm_final = None

    # long_way = False

    # # Проверяем результат
    # if short_solution.success:
    #     Vm = short_solution.x
    #     print("Решение найдено: Vm =", Vm)
    #     if np.linalg.norm(Vm) > Vmax or not short_solution.success:
    #         print("Vm > максимально возможного, ищу решение для 2 случая")
    #         long_way = True
    #         ang_initial = np.arctan2(r[1], r[0])
    #         time_initial = time()
    #         long_solution = least_squares(long_dist, ang_initial, bounds=(-np.pi, np.pi))
    #         time_final = time()

    #         print("Время поиска 2 решения:", (time_final - time_initial) * 1000)
    #         if long_solution.success:
    #             ang = long_solution.x[0]
    #             print("Решение 2 найдено: ang =", ang)
    #             Vm_final = np.array([Vmax * np.cos(ang), Vmax * np.sin(ang)])
    #         else:
    #             print("Решение 2 не найдено.")
    #     else:
    #         Vm_final = Vm
    # else:
    # print("Решение не найдено.")
    Vm_final = None

    long_way = True

    ang_initial = np.arctan2(r[1], r[0])

    time_initial = time()
    long_solution = least_squares(long_dist, ang_initial, xtol=ang_tolerance, ftol=cos_tolerance, method='lm')
    time_final = time()

    print("Время поиска длинного решения:", (time_final - time_initial) * 1000, "мс")

    if abs(long_solution.fun) > cos_tolerance:
        long_way = False
        print("Длинное решение не найдено")
        Vm_initial = r / np.linalg.norm(r) * Vmax

        time_initial = time()
        short_solution = least_squares(short_dist, Vm_initial, xtol=v_tolerance, ftol=r_tolerance, method='lm')
        time_final = time()

        print("Время поиска короткого решения:", (time_final - time_initial) * 1000, "мс")

        if np.linalg.norm(short_solution.fun) > r_tolerance:
            print("Короткое решение не найдено")
        else:
            Vm_final = short_solution.x
            print("Короткое решение найдено", Vm_final)
    else:
        ang = long_solution.x[0]
        print("Длинное решение найдено", ang)
        Vm_final = np.array([Vmax * np.cos(ang), Vmax * np.sin(ang)])


    # print("Время поиска длинного решения:", (time_final - time_initial) * 1000)

    # print(Vm_final, long_way)

    print(Vm_final - Va)

    if Vm_final is not None:
        way = []
        t1 = np.linalg.norm(Vm_final - Va) / Am
        t2 = np.linalg.norm(r - ((Vm_final + Va) * np.linalg.norm(Vm_final - Va) + (Vm_final + Vb) * np.linalg.norm(Vm_final - Vb)) / (2 * Am)) / Vmax + t1
        t3 = t2 + np.linalg.norm(Vm_final - Vb) / Am
        print("Время разгона робота =", t1)
        if long_way:
            print("Время РПД робота =", t2 - t1)
        print("Время торможения робота =", t3 - t2)
        print("Общее время =", t3)
        for i in range(int(1 / delay)):
            t = t3 * i * delay
            if t < t1:
                a = Am * (Vm_final - Va) / np.linalg.norm(Vm_final - Va)
                now_r = Va * t + a * t ** 2 / 2
            elif t < t2:
                now_r = (Vm_final + Va) * np.linalg.norm(Vm_final - Va) / (2 * Am) + (t - t1) * Vm_final
            else:
                a = Am * (Vb - Vm_final) / np.linalg.norm(Vb - Vm_final)
                now_r = r - (Vm_final + Vb) * np.linalg.norm(Vm_final - Vb) / (2 * Am) + Vm_final * (t - t2) + a * (t - t2) ** 2 / 2
            way.append(now_r)
        way.append(r)
        plot_way(way)