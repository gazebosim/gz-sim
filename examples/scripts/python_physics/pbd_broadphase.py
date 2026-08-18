import math

class SpatialHashGrid:
    def __init__(self, cell_size=2.0):
        self.cell_size = cell_size
        self.grid = {}

    def _hash(self, point):
        return (
            math.floor(point[0] / self.cell_size),
            math.floor(point[1] / self.cell_size),
            math.floor(point[2] / self.cell_size)
        )

    def add(self, entity, aabb_min, aabb_max):
        min_cell = self._hash(aabb_min)
        max_cell = self._hash(aabb_max)

        for x in range(min_cell[0], max_cell[0] + 1):
            for y in range(min_cell[1], max_cell[1] + 1):
                for z in range(min_cell[2], max_cell[2] + 1):
                    key = (x, y, z)
                    if key not in self.grid:
                        self.grid[key] = []
                    self.grid[key].append(entity)

    def get_candidate_pairs(self):
        pairs = set()
        for cell, entities in self.grid.items():
            if len(entities) > 1:
                for i in range(len(entities)):
                    for j in range(i + 1, len(entities)):
                        e1 = entities[i]
                        e2 = entities[j]
                        if e1 != e2:
                            # Store sorted tuple to avoid duplicates (e1, e2) and (e2, e1)
                            pairs.add(tuple(sorted((int(e1), int(e2)))))
        return pairs

    def clear(self):
        self.grid.clear()
