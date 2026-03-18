import random
import heapq
import matplotlib.pyplot as plt
from collections import deque
import time

# -----------------------------
# Grid Generation
# -----------------------------
ROWS = 10
COLS = 10
GRID = []
START = (0,0)
GOAL = (ROWS-1,COLS-1)

TERRAINS = ['.','~','&','#']  # normal, water, ink, obstacle
WEIGHTS = [0.6,0.15,0.15,0.1]  # probabilities

for r in range(ROWS):
    row = []
    for c in range(COLS):
        if (r,c) == START:
            row.append('S')
        elif (r,c) == GOAL:
            row.append('G')
        else:
            row.append(random.choices(TERRAINS, WEIGHTS)[0])
    GRID.append(row)

# -----------------------------
# Utility Functions
# -----------------------------
DIRECTIONS = [(-1,0),(1,0),(0,-1),(0,1)]

def get_mov_cost(cell):
    if cell == '.': return 1
    elif cell == '~': return 4
    elif cell == '&': return 2
    elif cell in ['S','G']: return 1
    return 1

def claustrophobic(r,c):
    pillar_count = 0
    for dr,dc in DIRECTIONS:
        nr,nc = r+dr,c+dc
        if 0<=nr<ROWS and 0<=nc<COLS:
            if GRID[nr][nc] == '#': pillar_count += 1
    return pillar_count <= 2

def update_ink(cell, ink_count, penalty):
    if cell == '~':
        ink_count += 1
        if ink_count == 3: penalty = True
    elif cell == '.':
        ink_count = 0
        penalty = False
    return ink_count, penalty

def heuristic(r,c):
    return abs(r-GOAL[0]) + abs(c-GOAL[1])

def draw_path(grid, path):
    temp = [row.copy() for row in grid]
    for r,c in path:
        if temp[r][c] not in ['S','G']: temp[r][c]='p'
    print('\n')
    for row in temp: print(' '.join(row))
    print('\n')

# -----------------------------
# BFS
# -----------------------------
def BFS():
    start_time=time.time()
    queue = deque([(START[0], START[1], [(START[0], START[1])])])
    visited = set([START])
    while queue:
        r,c,path = queue.popleft()
        if (r,c) == GOAL:
            return path, len(path)-1, time.time()-start_time
        for dr,dc in DIRECTIONS:
            nr,nc = r+dr,c+dc
            if 0<=nr<ROWS and 0<=nc<COLS and GRID[nr][nc]!='#' and (nr,nc) not in visited:
                visited.add((nr,nc))
                queue.append((nr,nc,path+[(nr,nc)]))
    return None,None, time.time()-start_time

# -----------------------------
# DFS
# -----------------------------
def DFS():
    start_time=time.time()
    stack = [(START[0], START[1], [(START[0], START[1])])]
    visited = set([START])
    while stack:
        r,c,path = stack.pop()
        if (r,c) == GOAL:
            return path, len(path)-1, time.time()-start_time
        for dr,dc in DIRECTIONS:
            nr,nc = r+dr,c+dc
            if 0<=nr<ROWS and 0<=nc<COLS and GRID[nr][nc]!='#' and (nr,nc) not in visited:
                visited.add((nr,nc))
                stack.append((nr,nc,path+[(nr,nc)]))
    return None,None, time.time()-start_time

# -----------------------------
# Iterative Deepening Search
# -----------------------------
def IDS():
    start_time=time.time()
    def DLS(r,c,path,depth,visited):
        if depth==0 and (r,c)==GOAL: return path
        if depth>0:
            for dr,dc in DIRECTIONS:
                nr,nc = r+dr,c+dc
                if 0<=nr<ROWS and 0<=nc<COLS and GRID[nr][nc]!='#' and (nr,nc) not in visited:
                    visited.add((nr,nc))
                    res = DLS(nr,nc,path+[(nr,nc)],depth-1,visited)
                    if res: return res
                    visited.remove((nr,nc))
        return None

    max_depth = ROWS*COLS
    for depth in range(max_depth):
        visited = set([START])
        path = DLS(START[0],START[1],[START],depth,visited)
        if path: return path,len(path)-1,time.time()-start_time
    return None,None,time.time()-start_time

# -----------------------------
# Uniform Cost Search
# -----------------------------
def UCS():
    start_time=time.time()
    pq = []
    heapq.heappush(pq,(0,START[0],START[1],0,False,[START]))
    visited = set()
    while pq:
        cost,r,c,ink,penalty,path = heapq.heappop(pq)
        state=(r,c,ink,penalty)
        if state in visited: continue
        visited.add(state)
        if (r,c)==GOAL: return path,cost,time.time()-start_time
        for dr,dc in DIRECTIONS:
            nr,nc = r+dr,c+dc
            if 0<=nr<ROWS and 0<=nc<COLS and GRID[nr][nc]!='#':
                cell = GRID[nr][nc]
                new_ink,new_penalty=update_ink(cell,ink,penalty)
                new_cost=cost+get_mov_cost(cell)
                if new_penalty: new_cost+=2
                heapq.heappush(pq,(new_cost,nr,nc,new_ink,new_penalty,path+[(nr,nc)]))
    return None,None,time.time()-start_time

# -----------------------------
# A* Search
# -----------------------------
def A_star():
    start_time=time.time()
    pq=[]
    heapq.heappush(pq,(0,0,START[0],START[1],0,False,[START]))
    visited=set()
    while pq:
        f,g,r,c,ink,penalty,path = heapq.heappop(pq)
        state=(r,c,ink,penalty)
        if state in visited: continue
        visited.add(state)
        if (r,c)==GOAL: return path,g,time.time()-start_time
        for dr,dc in DIRECTIONS:
            nr,nc = r+dr,c+dc
            if 0<=nr<ROWS and 0<=nc<COLS and GRID[nr][nc]!='#' and claustrophobic(nr,nc):
                cell = GRID[nr][nc]
                new_ink,new_penalty=update_ink(cell,ink,penalty)
                move_cost=get_mov_cost(cell)
                if new_penalty: move_cost+=2
                new_g=g+move_cost
                new_f=new_g+heuristic(nr,nc)
                heapq.heappush(pq,(new_f,new_g,nr,nc,new_ink,new_penalty,path+[(nr,nc)]))
    return None,None,time.time()-start_time

# -----------------------------
# Greedy Best First Search
# -----------------------------
def GBFS():
    start_time=time.time()
    pq=[]
    heapq.heappush(pq,(heuristic(*START),START[0],START[1],[START]))
    visited=set([START])
    while pq:
        h,r,c,path = heapq.heappop(pq)
        if (r,c)==GOAL: return path,len(path)-1,time.time()-start_time
        for dr,dc in DIRECTIONS:
            nr,nc=r+dr,c+dc
            if 0<=nr<ROWS and 0<=nc<COLS and GRID[nr][nc]!='#' and (nr,nc) not in visited and claustrophobic(nr,nc):
                visited.add((nr,nc))
                heapq.heappush(pq,(heuristic(nr,nc),nr,nc,path+[(nr,nc)]))
    return None,None,time.time()-start_time

# -----------------------------
# Beam Search
# -----------------------------
def Beam_Search():
    start_time=time.time()
    beam_width=3
    nodes=[(START[0],START[1],0,False,[START])]
    while nodes:
        next_nodes=[]
        for r,c,ink,penalty,path in nodes:
            if (r,c)==GOAL: return path,len(path)-1,time.time()-start_time
            for dr,dc in DIRECTIONS:
                nr,nc=r+dr,c+dc
                if 0<=nr<ROWS and 0<=nc<COLS and GRID[nr][nc]!='#' and claustrophobic(nr,nc):
                    new_ink,new_penalty=update_ink(GRID[nr][nc],ink,penalty)
                    next_nodes.append((nr,nc,new_ink,new_penalty,path+[(nr,nc)]))
        next_nodes.sort(key=lambda x: heuristic(x[0],x[1]))
        nodes=next_nodes[:beam_width]
    return None,None,time.time()-start_time

# -----------------------------
# Run All Algorithms and Compare
# -----------------------------
algorithms = [('BFS',BFS),('DFS',DFS),('IDS',IDS),('UCS',UCS),('A*',A_star),('GBFS',GBFS),('Beam',Beam_Search)]
results = {}

for name,func in algorithms:
    path,cost,elapsed=func()
    results[name]={'path':path,'cost':cost,'time':elapsed}
    print(f'--- {name} ---')
    print('Path:', path)
    print('Cost/Length:', cost)
    print(f'Time Taken: {elapsed:.6f} sec')
    if path: draw_path(GRID,path)

# -----------------------------
# Plot Comparison Graphs
# -----------------------------
# Cost/Length Comparison
plt.figure(figsize=(12,5))
plt.subplot(1,2,1)
names=[]
costs=[]
for name in results:
    names.append(name)
    cost = results[name]['cost'] if results[name]['cost'] is not None else 0
    costs.append(cost)
plt.bar(names,costs,color='skyblue')
plt.ylabel('Path Cost / Length')
plt.title('Pathfinding Cost/Length Comparison')

# Time Comparison
plt.subplot(1,2,2)
times=[]
for name in results:
    times.append(results[name]['time'])
plt.bar(names,times,color='salmon')
plt.ylabel('Time (seconds)')
plt.title('Pathfinding Time Comparison')
plt.tight_layout()
plt.show()