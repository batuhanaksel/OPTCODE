import random
import time
from ortools.linear_solver import pywraplp

LP_COST_CACHE = {}
GROUP_COUNT = 1
RUNWAY_COUNT = 1
# Gruplama modu:
# "T" : target time T_i'ye göre
# "APPEAR" : appearance time a_i'ye göre
# "EARLIEST" : earliest time E_i'ye göre
# "LATEST" : latest time L_i'ye göre
GROUP_MODE = "APPEAR"
COMBINED_ALPHA = 0.01


class AircraftData:
    def __init__(self, n_planes,
                 appearance, earliest, target, latest,
                 pen_early, pen_late, separation):
        self.n = n_planes
        self.appearance = appearance
        self.earliest = earliest
        self.target = target
        self.latest = latest
        self.pen_early = pen_early
        self.pen_late = pen_late
        self.sep = separation


dataset = """
 44 30
 0 0 0 0 2.00 2.00 
 99999 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 4 79 137 196 2.00 2.00 
 96 99999 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 71 146 271 396 1.00 1.00 
 72 72 99999 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 151 226 351 476 1.00 1.00 
 72 72 80 99999 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 231 306 431 556 1.00 1.00 
 72 72 80 80 99999 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 303 378 503 628 2.00 2.00 
 96 96 200 200 200 99999 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 453 528 626 724 2.00 2.00 
 96 96 200 200 200 96 99999 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 560 635 727 820 2.00 2.00 
 96 96 200 200 200 96 96 99999 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 695 770 895 1020 1.00 1.00 
 72 72 80 80 80 72 72 72 
 99999 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 767 842 967 1092 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 99999 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 967 1042 1167 1292 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 99999 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 1039 1114 1239 1364 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 99999 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 1239 1314 1439 1564 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 99999 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 1319 1394 1519 1644 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 99999 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 1391 1466 1591 1716 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 99999 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 1591 1666 1791 1916 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 99999 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 1671 1746 1871 1996 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 99999 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 1743 1818 1943 2068 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 99999 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 1943 2018 2143 2268 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 99999 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 2023 2098 2223 2348 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 99999 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 2095 2170 2295 2420 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 99999 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 2191 2266 2391 2516 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 99999 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 2391 2466 2591 2716 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 99999 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 2463 2538 2663 2788 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 99999 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 2663 2738 2863 2988 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 99999 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 2743 2818 2943 3068 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 99999 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 2823 2898 3023 3148 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 99999 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 2903 2978 3103 3228 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 99999 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 2975 3050 3175 3300 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 99999 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 3071 3146 3271 3396 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 99999 200 96 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 3271 3346 3471 3596 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 99999 72 
 80 72 72 80 80 80 72 72 
 72 72 72 80 
 3343 3418 3543 3668 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 99999 
 200 96 96 200 200 200 96 96 
 96 96 96 200 
 3543 3618 3743 3868 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 99999 72 72 80 80 80 72 72 
 72 72 72 80 
 3615 3690 3815 3940 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 99999 96 200 200 200 96 96 
 96 96 96 200 
 3711 3786 3911 4036 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 99999 200 200 200 96 96 
 96 96 96 200 
 3911 3986 4111 4236 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 99999 80 80 72 72 
 72 72 72 80 
 3991 4066 4191 4316 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 99999 80 72 72 
 72 72 72 80 
 4071 4146 4271 4396 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 99999 72 72 
 72 72 72 80 
 4143 4218 4343 4468 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 99999 96 
 96 96 96 200 
 4239 4314 4439 4564 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 99999 
 96 96 96 200 
 4335 4410 4535 4660 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 99999 96 96 200 
 4431 4506 4631 4756 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 99999 96 200 
 4527 4602 4727 4852 2.00 2.00 
 96 96 200 200 200 96 96 96 
 200 96 200 96 200 200 96 200 
 200 96 200 200 96 96 200 96 
 200 200 200 200 96 96 200 96 
 200 96 96 200 200 200 96 96 
 96 96 99999 200 
 4727 4802 4927 5052 1.00 1.00 
 72 72 80 80 80 72 72 72 
 80 72 80 72 80 80 72 80 
 80 72 80 80 72 72 80 72 
 80 80 80 80 72 72 80 72 
 80 72 72 80 80 80 72 72 
 72 72 72 99999 
 
         
"""


def load_instance_from_string(s: str) -> AircraftData:
    print(">> Data set okunuyor...")
    tokens = s.split()
    it = iter(tokens)

    n_planes = int(next(it))
    _ = next(it)

    print(f"   Uçak sayısı = {n_planes}")

    appearance = []
    earliest = []
    target = []
    latest = []
    pen_early = []
    pen_late = []

    sep = [[0.0] * n_planes for _ in range(n_planes)]

    for i in range(n_planes):
        a = float(next(it))
        e = float(next(it))
        t = float(next(it))
        l = float(next(it))
        pe = float(next(it))
        pl = float(next(it))

        appearance.append(a)
        earliest.append(e)
        target.append(t)
        latest.append(l)
        pen_early.append(pe)
        pen_late.append(pl)

        print(f"   Uçak {i+1}: a={a}, E={e}, T={t}, L={l}, pe={pe}, pl={pl}")

        for j in range(n_planes):
            sep[i][j] = float(next(it))
        print(f"     -> sep[{i+1}][1..3] = {sep[i][0:3]} ...")

    print()

    return AircraftData(n_planes,
                        appearance, earliest, target, latest,
                        pen_early, pen_late, sep)


def split_order_to_runways(order, runway_count: int):
    R = max(1, int(runway_count))
    runways = [[] for _ in range(R)]
    for idx, plane in enumerate(order):
        runways[idx % R].append(plane)
    return runways


def greedy_schedule(order, data: AircraftData):
    if RUNWAY_COUNT <= 1:
        n = data.n
        t = [-1.0] * n

        first = order[0]
        t[first] = max(data.earliest[first], data.appearance[first])

        if t[first] > data.latest[first]:
            return False, None

        prev = first

        for k in range(1, len(order)):
            i = order[k]
            earliest_feasible = max(
                data.earliest[i],
                data.appearance[i],
                t[prev] + data.sep[prev][i]
            )
            t[i] = earliest_feasible

            if t[i] > data.latest[i]:
                return False, None

            prev = i

        return True, t

    orders_by_r = split_order_to_runways(order, RUNWAY_COUNT)
    n = data.n
    t = [-1.0] * n

    used = set()
    for r_order in orders_by_r:
        for p in r_order:
            used.add(p)

        if not r_order:
            continue

        first = r_order[0]
        t[first] = max(data.earliest[first], data.appearance[first])
        if t[first] > data.latest[first]:
            return False, None

        prev = first
        for k in range(1, len(r_order)):
            i = r_order[k]
            earliest_feasible = max(
                data.earliest[i],
                data.appearance[i],
                t[prev] + data.sep[prev][i]
            )
            t[i] = earliest_feasible
            if t[i] > data.latest[i]:
                return False, None
            prev = i

    if len(used) != n:
        return False, None

    return True, t


def solve_t_for_order(order, data: AircraftData):
    if RUNWAY_COUNT <= 1:
        solver = pywraplp.Solver.CreateSolver('CBC')
        if solver is None:
            raise RuntimeError("CBC solver bulunamadı")

        n = data.n
        inf = solver.infinity()

        t = [solver.NumVar(data.earliest[i], data.latest[i], f"t_{i}") for i in range(n)]
        e = [solver.NumVar(0.0, inf, f"e_{i}") for i in range(n)]
        l = [solver.NumVar(0.0, inf, f"l_{i}") for i in range(n)]

        for i in range(n):
            solver.Add(t[i] == data.target[i] - e[i] + l[i])

        for k in range(len(order) - 1):
            i = order[k]
            j = order[k + 1]
            solver.Add(t[j] >= t[i] + data.sep[i][j])

        obj = solver.Objective()
        for i in range(n):
            obj.SetCoefficient(e[i], data.pen_early[i])
            obj.SetCoefficient(l[i], data.pen_late[i])
        obj.SetMinimization()

        status = solver.Solve()
        if status not in (pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE):
            return float("inf"), None

        t_sol = [t[i].solution_value() for i in range(n)]
        cost = obj.Value()
        return cost, t_sol

    orders_by_r = split_order_to_runways(order, RUNWAY_COUNT)

    solver = pywraplp.Solver.CreateSolver('CBC')
    if solver is None:
        raise RuntimeError("CBC solver bulunamadı")

    n = data.n
    inf = solver.infinity()

    t = [solver.NumVar(data.earliest[i], data.latest[i], f"t_{i}") for i in range(n)]
    e = [solver.NumVar(0.0, inf, f"e_{i}") for i in range(n)]
    l = [solver.NumVar(0.0, inf, f"l_{i}") for i in range(n)]

    for i in range(n):
        solver.Add(t[i] == data.target[i] - e[i] + l[i])
        solver.Add(t[i] >= data.appearance[i])

    for r_order in orders_by_r:
        for k in range(len(r_order) - 1):
            i = r_order[k]
            j = r_order[k + 1]
            solver.Add(t[j] >= t[i] + data.sep[i][j])

    obj = solver.Objective()
    for i in range(n):
        obj.SetCoefficient(e[i], data.pen_early[i])
        obj.SetCoefficient(l[i], data.pen_late[i])
    obj.SetMinimization()

    status = solver.Solve()
    if status not in (pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE):
        return float("inf"), None

    t_sol = [t[i].solution_value() for i in range(n)]
    cost = obj.Value()
    return cost, t_sol


def schedule_cost(order, data: AircraftData):
    key = tuple(order)

    if key in LP_COST_CACHE:
        return LP_COST_CACHE[key]

    feasible, _ = greedy_schedule(order, data)
    if not feasible:
        LP_COST_CACHE[key] = float("inf")
        return float("inf")

    cost, _ = solve_t_for_order(order, data)
    LP_COST_CACHE[key] = cost
    return cost


def construct_initial_solution(data: AircraftData):
    n = data.n
    indices = list(range(n))
    perm = sorted(indices, key=lambda i: data.target[i])

    cost, t_sol = solve_t_for_order(perm, data)

    print(">> Başlangıç çözümü (T'ye göre sıralı):", [i + 1 for i in perm])
    print(f">> Başlangıç cost (LP): {cost:.2f}\n")

    return perm, t_sol, cost


def two_opt_move(order, i, j):
    return order[:i] + order[i:j + 1][::-1] + order[j + 1:]


def make_plane_groups_sorted_key(data: AircraftData, group_count: int, key_func):
    n = data.n
    g = min(group_count, n)
    if g <= 1:
        return [0] * n, 1

    sorted_planes = sorted(range(n), key=key_func)

    base_size = n // g
    remainder = n % g

    plane_to_group = [0] * n
    idx = 0
    for grp in range(g):
        size = base_size + (1 if grp < remainder else 0)
        for _ in range(size):
            if idx >= n:
                break
            p = sorted_planes[idx]
            plane_to_group[p] = grp
            idx += 1

    return plane_to_group, g


def make_plane_groups_combined_t_sep(data: AircraftData, group_count: int, alpha: float):
    n = data.n
    g = min(group_count, n)
    if g <= 1:
        return [0] * n, 1

    avg_sep = []
    for i in range(n):
        s = 0.0
        for j in range(n):
            s += data.sep[i][j]
        avg_sep.append(s / n)

    def key_func(i):
        return data.target[i] + alpha * avg_sep[i]

    return make_plane_groups_sorted_key(data, g, key_func)


def build_plane_groups(data: AircraftData, group_count: int, mode: str):
    mode = mode.upper()
    if mode == "T":
        return make_plane_groups_sorted_key(data, group_count, key_func=lambda i: data.target[i])
    elif mode == "APPEAR":
        return make_plane_groups_sorted_key(data, group_count, key_func=lambda i: data.appearance[i])
    elif mode == "EARLIEST":
        return make_plane_groups_sorted_key(data, group_count, key_func=lambda i: data.earliest[i])
    elif mode == "LATEST":
        return make_plane_groups_sorted_key(data, group_count, key_func=lambda i: data.latest[i])
    elif mode == "PENALTY_LATE":
        return make_plane_groups_sorted_key(data, group_count, key_func=lambda i: data.pen_late[i])
    elif mode == "COMBINED_T_SEP":
        return make_plane_groups_combined_t_sep(data, group_count, COMBINED_ALPHA)
    else:
        return [0] * data.n, 1


def build_position_groups(n, freeze, group_count):
    effective_len = n - freeze
    if effective_len <= 1 or group_count <= 1:
        return [(freeze, n - 1)]

    g = min(group_count, effective_len)
    base_size = effective_len // g
    remainder = effective_len % g

    groups = []
    start = freeze
    for k in range(g):
        size = base_size + (1 if k < remainder else 0)
        end = start + size - 1
        groups.append((start, end))
        start = end + 1

    return groups


def best_improvement_2opt(order, data: AircraftData,
                          grouping_mode: str,
                          group_count: int,
                          plane_to_group=None,
                          real_group_count=None):
    n = len(order)
    freeze = 0
    iteration = 0

    start_2opt = time.perf_counter()

    grouping_mode = grouping_mode.upper()

    while True:
        iter_start = time.perf_counter()

        improved = False
        best_cost = schedule_cost(order, data)
        best_order = order

        if grouping_mode == "POSITION":
            groups = build_position_groups(n, freeze, group_count)

            for (start_idx, end_idx) in groups:
                if end_idx <= start_idx:
                    continue
                for i in range(start_idx, end_idx):
                    for j in range(i + 1, end_idx + 1):
                        cand = two_opt_move(order, i, j)
                        cand_cost = schedule_cost(cand, data)
                        if cand_cost < best_cost:
                            best_cost = cand_cost
                            best_order = cand
                            improved = True
        else:
            gcount = real_group_count if real_group_count is not None else group_count
            group_positions = [[] for _ in range(gcount)]
            for pos in range(freeze, n):
                plane = order[pos]
                g = plane_to_group[plane]
                group_positions[g].append(pos)

            for g in range(gcount):
                positions = group_positions[g]
                m = len(positions)
                if m <= 1:
                    continue
                positions.sort()
                for a in range(m):
                    i = positions[a]
                    for b in range(a + 1, m):
                        j = positions[b]
                        cand = two_opt_move(order, i, j)
                        cand_cost = schedule_cost(cand, data)
                        if cand_cost < best_cost:
                            best_cost = cand_cost
                            best_order = cand
                            improved = True

        iter_end = time.perf_counter()
        print(f"  [2-opt] Iterasyon {iteration} süresi: {iter_end - iter_start:.4f} saniye")

        if not improved:
            break

        order = best_order
        iteration += 1

    end_2opt = time.perf_counter()
    print(f"\n  [2-opt] TOPLAM 2-opt süresi: {end_2opt - start_2opt:.4f} saniye\n")

    return order, best_cost


def run_heuristic(data: AircraftData):
    global LP_COST_CACHE
    LP_COST_CACHE = {}

    alpha_order, t_init, alpha_cost = construct_initial_solution(data)

    grouping_mode = GROUP_MODE.upper()

    if grouping_mode == "POSITION":
        plane_to_group = None
        real_group_count = None
    else:
        plane_to_group, real_group_count = build_plane_groups(
            data, GROUP_COUNT, grouping_mode
        )
        print(">> Plane-based grup atamaları (mode =", grouping_mode, "):")
        for g in range(real_group_count):
            members = [i + 1 for i in range(data.n) if plane_to_group[i] == g]
            print(f"  Grup {g}: uçaklar (1-based) = {members}")
        print()

    alpha_order, alpha_cost = best_improvement_2opt(
        alpha_order,
        data,
        grouping_mode=grouping_mode,
        group_count=GROUP_COUNT,
        plane_to_group=plane_to_group,
        real_group_count=real_group_count
    )

    _, best_t = solve_t_for_order(alpha_order, data)
    return alpha_order, best_t, alpha_cost


def compute_lp_relaxation_lower_bound(data: AircraftData):
    print(">> LP relaksasyon lower bound modeli kuruluyor...")

    solver = pywraplp.Solver.CreateSolver('CBC')
    if solver is None:
        raise RuntimeError("CBC solver bulunamadı (LB).")

    n = data.n
    inf = solver.infinity()

    t = [solver.NumVar(data.earliest[i], data.latest[i], f"t_lb_{i}") for i in range(n)]
    e = [solver.NumVar(0.0, inf, f"e_lb_{i}") for i in range(n)]
    l = [solver.NumVar(0.0, inf, f"l_lb_{i}") for i in range(n)]

    for i in range(n):
        solver.Add(t[i] == data.target[i] - e[i] + l[i])

    minE = min(data.earliest)
    maxL = max(data.latest)
    max_sep = max(data.sep[i][j] for i in range(n) for j in range(n))
    M = (maxL - minE) + max_sep

    x = {}
    for i in range(n):
        for j in range(i + 1, n):
            x[(i, j)] = solver.NumVar(0.0, 1.0, f"x_{i}_{j}")

    for i in range(n):
        for j in range(i + 1, n):
            x_ij = x[(i, j)]
            solver.Add(t[j] >= t[i] + data.sep[i][j] - M * (1 - x_ij))
            solver.Add(t[i] >= t[j] + data.sep[j][i] - M * (x_ij))

    obj = solver.Objective()
    for i in range(n):
        obj.SetCoefficient(e[i], data.pen_early[i])
        obj.SetCoefficient(l[i], data.pen_late[i])
    obj.SetMinimization()

    status = solver.Solve()
    if status not in (pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE):
        print(">> LP relaksasyon çözüm bulamadı (infeasible ya da error).")
        return float("inf")

    lb_value = obj.Value()
    print(f">> LP relaksasyon lower bound = {lb_value:.4f}")
    return lb_value


if __name__ == "__main__":
    random.seed(0)

    data = load_instance_from_string(dataset)

    start = time.perf_counter()
    best_order, best_t, best_cost = run_heuristic(data)
    end = time.perf_counter()

    print(f"\n----- HEURISTIC SONUÇ ({data.n} uçak, runway={RUNWAY_COUNT}) -----")
    print("En iyi sıra (1-based):", [i + 1 for i in best_order])

    if RUNWAY_COUNT > 1:
        orders_by_r = split_order_to_runways(best_order, RUNWAY_COUNT)
        for r, od in enumerate(orders_by_r):
            print(f"  Runway {r} sıra (1-based): {[i + 1 for i in od]}")

    if best_t is not None:
        print("İniş zamanları t_i (uçak index 0-based):")
        for i, t_val in enumerate(best_t):
            print(f"  Uçak {i+1}: t = {t_val:.2f}")
    print("Heuristic toplam ceza:", best_cost)
    print(f"Heuristic toplam çalışma süresi: {end - start:.4f} saniye\n")

    lb = compute_lp_relaxation_lower_bound(data)
    print("\n----- LOWER BOUND ANALİZİ -----")
    print(f"Lower bound (LP relaksasyon): {lb:.4f}")
    if lb == float("inf"):
        print("LB hesaplanamadı, gap yorumlanamıyor.")
    else:
        abs_gap = best_cost - lb
        print(f"Mutlak fark (Heuristic - LB): {abs_gap:.4f}")
        if lb > 1e-6:
            rel_gap = abs_gap / lb * 100.0
            print(f"Relatif fark (Heuristic/LB - 1): %{rel_gap:.2f}")
        else:
            print("LB ≈ 0 olduğu için relatif gap anlamlı değil.")
