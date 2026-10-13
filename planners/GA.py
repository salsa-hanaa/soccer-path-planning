import math
import random

from environment.planner_base import BasePlanner
from environment.state import WorldState

PI2 = 2*math.pi
K = 3                          # langkah lookahead = panjang genom
GEN = 4                        # generasi per tick
N_ANGLE, N_SPEED = 12, 10       # populasi awal = N_ANGLE x N_SPEED
MIN_SPEED, MAX_SPEED = 0.5, 210.0 
SIG_ANGLE, SIG_SPEED = 0.35, 25.0 # skala mutasi angle & speed pada gen
SEL_PRESSURE = 4.0               # Tekanan seleksi high fitness
MAX_DT = 3                       # Batas selisih waktu update span
SPAN_TOL = 10                    # Toleransi span enemy
EX_TOL = 15                      # Toleransi deteksi enemy
N_ENEMY = 3


H = 0.2                        # durasi satu langkah genom (detik)
TAU = 0.4                      # konstanta lag respons
SUB = 4                        # Jumlah sub-langkah simulasi per langkah genom
DTS = H / SUB                  # Durasi 1 sub langkah
V_CAP = 220.0                   
S_SAT = 60.0                   # Saturasi tanh (jenuh mulai 60)
BALL_R = 10.0                  

# bobot fitness
SAFE_MARGIN = 50.0 
SIG_E = 12.0                    # Kelandaian risk enemy
SIG_C = 30.0                    # Lebar gaussian contact
W_BALL, W_FACE, W_ENEMY, W_SPEED = 2.0, 3.5, 2.5, 8.0
W_INFO, SIG_FOV, T_SCAN = 5.0, 0.5, 3.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def fold(y, lo, hi):
    span = max(hi - lo, 1e-6)
    t = (y - lo) % (2*span)
    return lo + (t if t <= span else 2*span - t)


def span_enemy(enemy, obs, state):
    x, y, vx, vy, last_time, r, span = enemy
    dt = state.time - last_time
    lo, hi = span

    lo = min(lo, obs.y, y)
    hi = max(hi, obs.y, y)

    if 0 < dt < MAX_DT:
        path = abs(vy) * dt
        sgn = 1.0 if vy > 0 else -1.0
        proj = (obs.y - y) * sgn
        if proj < path - SPAN_TOL:
            if sgn > 0:
                hi = max(hi, (path + y + obs.y) / 2)
            else:
                lo = min(lo, (y + obs.y - path) / 2)

    return [obs.x, obs.y, obs.vx, obs.vy, state.time, obs.radius, [lo, hi]]


class GeneticAlgorithm(BasePlanner):
    name = "GA"

    def reset(self, state: WorldState) -> None:
        self.enemy_memory = []     # [x, y, vx, vy, last_time, r, [lo, hi]]
        self.enemy_vmax = []       # |vy| terbesar yang pernah teramati per musuh
        self.context = {}          
        self.population = []
        self.best = None

    def _update_enemies(self, state: WorldState):
        for e in state.enemies:
            for i, m in enumerate(self.enemy_memory):
                if abs(m[0] - e.x) < EX_TOL:
                    self.enemy_memory[i] = span_enemy(m, e, state)
                    self.enemy_vmax[i] = max(self.enemy_vmax[i], abs(e.vy))
                    break
            else:
                self.enemy_memory.append([e.x, e.y, e.vx, e.vy, state.time, e.radius, [e.y-60, e.y+60]])
                self.enemy_vmax.append(abs(e.vy))


    def _context(self, state: WorldState):
        paths = []
        for (x, y, vx, vy, t, r, (lo, hi)), vm in zip(self.enemy_memory, self.enemy_vmax):
            v = math.copysign(max(vm, abs(vy)), vy if vy != 0 else 1.0)
            age = state.time - t
            paths.append([(x, fold(y + v*(age + (n + 1)*DTS), lo, hi)) for n in range(K*SUB)])

        seen = len(self.enemy_memory)
        unknown = (abs(N_ENEMY - seen)**4 / N_ENEMY**2)         
        scan = unknown * math.exp(-state.time / T_SCAN)    # hadiah informasi memudar seiring waktu

        return dict(paths=paths, radii=[m[5] for m in self.enemy_memory],
            unknown=unknown, scan=scan,
            ball=(state.ball_x, state.ball_y),
            target=(state.target_x, state.target_y),
            goal=(state.field_length, state.field_width / 2))

    def fitness(self, population, state: WorldState):
        fit = []
        con = self.context
        rr, L, W = state.robot_radius, state.field_length, state.field_width
        bx, by = con["ball"]
        tx, ty = con["target"]
        gx, gy = con["goal"]
        k = 1.0 - math.exp(-DTS / TAU)

        for genome in population:
            x, y, th = state.robot_x, state.robot_y, state.robot_theta
            vx, vy = state.robot_vx, state.robot_vy
            total = 0.0

            for j, (angle, speed) in enumerate(genome):
                v_cmd = V_CAP * math.tanh(speed / S_SAT)
                cx, cy = math.cos(angle)*v_cmd, math.sin(angle)*v_cmd

                risk = 0.0
                for s in range(SUB):
                    vx += (cx - vx) * k
                    vy += (cy - vy) * k
                    th += wrap(angle - th) * k
                    x = max(rr, min(L - rr, x + vx*DTS))
                    y = max(rr, min(W - rr, y + vy*DTS))

                    n = j*SUB + s
                    for pts, er in zip(con["paths"], con["radii"]):
                        ex, ey = pts[n]
                        clear = math.hypot(x - ex, y - ey) - (rr + er)
                        risk += 1.0 / (1.0 + math.exp((clear - SAFE_MARGIN) / SIG_E))

                d_stop = math.hypot(tx - (x + vx*TAU), ty - (y + vy*TAU))
                prog = -math.hypot(d_stop, 1.0) / (V_CAP * H)

                d_tgt = math.hypot(tx - x, ty - y)
                contact = math.exp(-(d_tgt / SIG_C)**2)
                face = math.cos(th - math.atan2(gy - y, gx - x))

                pen_speed = con["unknown"] * (math.hypot(vx, vy) / V_CAP)**2

                cover = math.exp(-(wrap(th - math.atan2(by - y, bx - x)) / SIG_FOV)**2)
                total += (W_BALL*prog + W_FACE*contact*face
                          - W_ENEMY*risk - W_SPEED*pen_speed
                          + W_INFO*con["scan"]*cover)

            fit.append(total)
        return fit

    def reproduce(self, par1, par2, state: WorldState, scale=1.0):
        child = []
        for (a1, s1), (a2, s2) in zip(par1, par2):
            w = random.random()
            x_kombinasi = (w * math.cos(a1)) + ((1.0 - w) * math.cos(a2))
            y_kombinasi = (w * math.sin(a1)) + ((1.0 - w) * math.sin(a2))
            child_angle = math.atan2(y_kombinasi, x_kombinasi) + random.gauss(0, SIG_ANGLE*scale)
            speed = w*s1 + (1.0 - w)*s2 + random.gauss(0, SIG_SPEED*scale)
            child.append((child_angle, max(MIN_SPEED, min(MAX_SPEED, speed))))
        return child

    def project(self, population, gen, state: WorldState):
        fitness = self.fitness(population, state)
        if gen == 1:
            return fitness, population

        f_max, f_min = max(fitness), min(fitness)
        spread = (f_max - f_min) + 1e-9
        weights = [math.exp(SEL_PRESSURE * (f - f_max) / spread) for f in fitness]

        new_population = [population[fitness.index(f_max)]]          # elitisme
        while len(new_population) < len(population):
            par1, par2 = random.choices(population, weights=weights, k=2)
            new_population.append(self.reproduce(par1, par2, state, scale=gen/GEN))
        return self.project(new_population, gen - 1, state)

    def _init_population(self):
        offset = random.uniform(0, PI2 / N_ANGLE)
        rad_angles = [offset + i*PI2/N_ANGLE for i in range(N_ANGLE)]
        speeds = [MIN_SPEED + i*(MAX_SPEED - MIN_SPEED)/(N_SPEED - 1) for i in range(N_SPEED)]

        population = []
        for a in rad_angles:
            for s in speeds:                                  
                genome = [(a, s)]
                for _ in range(K - 1):
                    pa, ps = genome[-1]
                    genome.append((pa + random.gauss(0, SIG_ANGLE),
                                   max(MIN_SPEED, min(MAX_SPEED, ps + random.gauss(0, SIG_SPEED)))))
                population.append(genome)

        if self.best is not None:                             
            population.append(self.best[1:] + [self.best[-1]])
        return population

    def plan(self, state: WorldState) -> tuple[float, float]:
        if not hasattr(self, "enemy_memory"):
            self.reset(state)

        if state.ball_x is None:                              
            a = math.pi
            return state.robot_x + math.cos(a)*0, state.robot_y + math.sin(a)*0

        self._update_enemies(state)
        self.context = self._context(state)
        self.population = self._init_population()

        fitness, self.population = self.project(self.population, GEN, state)
        self.best = self.population[fitness.index(max(fitness))]

        best_angle, best_speed = self.best[0]
        return (state.robot_x + math.cos(best_angle) * best_speed,
                state.robot_y + math.sin(best_angle) * best_speed)