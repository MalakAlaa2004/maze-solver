import tkinter as tk
import os
from tkinter import messagebox, filedialog
from PIL import Image, ImageDraw, ImageTk
import heapq
from queue import PriorityQueue


class Node:
    def __init__(self, state, parent, action, g=0, h=0):
        self.state = state
        self.parent = parent
        self.action = action

        self.g = g  # Cost from start to this node
        self.h = h  # Heuristic estimate to the goal

    @property
    def f(self):
        return self.g + self.h  # Total cost function for A*

    def __lt__(self, other):
        return self.f < other.f

    def __lt__(self, other):
        return self.h < other.h

    def __eq__(self, other):
        return self.state == other.state

    def __hash__(self):
        return hash(str(self.state))


class GreedyBestFirstSearchFrontier:
    def __init__(self):
        self.frontier = PriorityQueue()

    def add(self, node, h):
        self.frontier.put((h, node))

    def contains_state(self, state):
        # Check if any node in the frontier has the given state
        return any(node.state == state for _, node in self.frontier.queue)

    def empty(self):
        return self.frontier.empty()

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            _, node = self.frontier.get()
            return node


class StackFrontier():
    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[-1]
            self.frontier = self.frontier[:-1]
            return node


class QueueFrontier(StackFrontier):
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node


class PriorityQueueFrontier:
    def __init__(self):
        self.queue = []
        self.count = 0  # Tie-breaker counter

    def add(self, node):
        priority = node.f
        entry = (priority, self.count, node)
        heapq.heappush(self.queue, entry)
        self.count += 1

    def remove(self):
        if self.empty():
            raise IndexError("Cannot remove from an empty queue")
        priority, _, node = heapq.heappop(self.queue)
        return node

    def contains_state(self, state):
        return any(node.state == state for _, _, node in self.queue)

    def empty(self):
        return len(self.queue) == 0

class Maze:

    def __init__(self, filename):

        with open(filename) as f:
            contents = f.read()

        if contents.count("A") != 1:
            raise Exception("maze must have exactly one start point")
        if contents.count("B") != 1:
            raise Exception("maze must have exactly one goal")

        # Determine height and width of maze
        contents = contents.splitlines()
        self.height = len(contents)
        self.width = max(len(line) for line in contents)

        # Keep track of walls
        self.walls = []
        for i in range(self.height):
            row = []
            for j in range(self.width):
                try:
                    if contents[i][j] == "A":
                        self.start = (i, j)
                        row.append(False)
                    elif contents[i][j] == "B":
                        self.goal = (i, j)
                        row.append(False)
                    elif contents[i][j] == " ":
                        row.append(False)
                    else:
                        row.append(True)
                except IndexError:
                    row.append(False)
            self.walls.append(row)

        self.solution = None

    def manhattan_distance(self, state, goal):
        """Compute the Manhattan distance between the current state and the goal."""
        return abs(state[0] - goal[0]) + abs(state[1] - goal[1])

    def print(self):
        solution = self.solution[1] if self.solution is not None else None
        print()
        for i, row in enumerate(self.walls):
            for j, col in enumerate(row):
                if col:
                    print("█", end="")
                elif (i, j) == self.start:
                    print("A", end="")
                elif (i, j) == self.goal:
                    print("B", end="")
                elif solution is not None and (i, j) in solution:
                    print("*", end="")
                else:
                    print(" ", end="")
            print()
        print()

    def neighbors(self, state):
        row, col = state
        candidates = [
            ("up", (row - 1, col)),
            ("down", (row + 1, col)),
            ("left", (row, col - 1)),
            ("right", (row, col + 1))
        ]

        result = []
        for action, (r, c) in candidates:
            if 0 <= r < self.height and 0 <= c < self.width and not self.walls[r][c]:
                result.append((action, (r, c)))
        return result

    def solve_dfs(self):
        """Finds a solution to maze, if one exists."""

        self.num_explored = 0

        start = Node(state=self.start, parent=None, action=None)
        frontier = StackFrontier()
        frontier.add(start)

        self.explored = set()

        while True:

            if frontier.empty():
                raise Exception("no solution")

            # Choose a node from the frontier
            node = frontier.remove()
            self.num_explored += 1

            # If node is the goal, then we have a solution
            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            # Mark node as explored
            self.explored.add(node.state)

            # Add neighbors to frontier
            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state) and state not in self.explored:
                    child = Node(state=state, parent=node, action=action)
                    frontier.add(child)

    def solve_bfs(self):
        """Finds a solution to maze, if one exists."""

        self.num_explored = 0

        start = Node(state=self.start, parent=None, action=None)
        frontier = QueueFrontier()
        frontier.add(start)

        self.explored = set()

        while True:

            if frontier.empty():
                raise Exception("no solution")

            # Choose a node from the frontier
            node = frontier.remove()
            self.num_explored += 1

            # If node is the goal, then we have a solution
            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            # Mark node as explored
            self.explored.add(node.state)

            # Add neighbors to frontier
            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state) and state not in self.explored:
                    child = Node(state=state, parent=node, action=action)
                    frontier.add(child)

    def HeuristicBFS(self):
        self.num_explored = 0

        start = Node(state=self.start, parent=None, action=None)
        frontier = GreedyBestFirstSearchFrontier()
        h = self.manhattan_distance(start.state, self.goal)
        frontier.add(start, h)

        self.explored = set()

        while True:

            if frontier.empty():
                raise Exception("no solution")

            # Choose a node from the frontier
            node = frontier.remove()
            self.num_explored += 1

            # If node is the goal, then we have a solution
            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            # Mark node as explored
            self.explored.add(node.state)

            # Add neighbors to frontier
            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state) and state not in self.explored:
                    child = Node(state=state, parent=node, action=action)
                    h = self.manhattan_distance(state, self.goal)
                    frontier.add(child, h)

    def HeuristicA(self):
        self.num_explored = 0

        start = Node(state=self.start, parent=None, action=None, g=0, h=self.manhattan_distance(self.start, self.goal))
        frontier = PriorityQueue()
        frontier.put((start.f, start))

        self.explored = set()

        while not frontier.empty():

            # Choose a node from the frontier
            _, node = frontier.get()
            self.num_explored += 1

            # If node is the goal, then we have a solution
            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            # Mark node as explored
            self.explored.add(node.state)

            # Add neighbors to frontier
            for action, state in self.neighbors(node.state):
                if state not in self.explored:
                    g = node.g + 1  # Assume cost between nodes is 1
                    h = self.manhattan_distance(state, self.goal)
                    child = Node(state=state, parent=node, action=action, g=g, h=h)
                    frontier.put((child.f, child))

    def solve_dijkstra(self):
        self.num_explored = 0

        start = Node(state=self.start, parent=None, action=None, g=0, h=0)
        frontier = PriorityQueueFrontier()
        frontier.add(start)

        self.explored = set()

        while True:

            if frontier.empty():
                raise Exception("no solution")

            # Choose a node from the frontier
            node = frontier.remove()
            self.num_explored += 1

            # If node is the goal, then we have a solution
            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            # Mark node as explored
            self.explored.add(node.state)

            # Add neighbors to frontier
            for action, state in self.neighbors(node.state):
                if state not in self.explored:
                    g = node.g + 1  # Assume cost between nodes is 1
                    h = self.manhattan_distance(state, self.goal)
                    child = Node(state=state, parent=node, action=action, g=g, h=h)
                    frontier.add(child)

    def solve_Bellman_Ford(self):

        self.num_explored = 0

        cost = {self.start: 0}
        parent = {self.start: None}
        queue = [self.start]
        self.explored = set()

        while queue:
            node = queue.pop(0)
            for action, state in self.neighbors(node):
                if state not in cost or cost[node] + 1 < cost[state]:
                    cost[state] = cost[node] + 1
                    parent[state] = node
                    queue.append(state)

        if self.goal not in cost:
            raise Exception("no solution")

        actions = []
        cells = []
        node = self.goal
        while node is not None:
            self.num_explored += 1
            self.explored.add(node)
            actions.append(parent[node])
            cells.append(node)
            node = parent[node]
        actions.reverse()
        cells.reverse()
        self.solution = (actions, cells)

    def solve_Floyd_Warshall(self):
        self.num_explored = 0
        self.explored = set()
        nodes = [(i, j) for i in range(self.height) for j in range(self.width) if not self.walls[i][j]]
        cost = {(i, j): float('inf') for i in range(self.height) for j in range(self.width) if not self.walls[i][j]}
        cost[self.start] = 0

        for _ in nodes:
            for i in nodes:
                for j in nodes:
                    if cost[i] + 1 < cost[j] and self.is_neighbor(i, j):
                        cost[j] = cost[i] + 1


        if cost[self.goal] == float('inf'):
            raise Exception("no solution")

        actions = []
        cells = []
        node = self.goal
        self.num_explored += 1

        while node != self.start:
            self.num_explored += 1
            self.explored.add(node)
            for action, state in self.neighbors(node):
                if cost[state] == cost[node] - 1:
                    actions.append(action)
                    cells.append(node)
                    node = state
                    break
        actions.reverse()
        cells.reverse()
        self.solution = (actions, cells)

    def is_neighbor(self, node1, node2):
        x1, y1 = node1
        x2, y2 = node2
        return abs(x1 - x2) + abs(y1 - y2) == 1 and not self.walls[x2][y2]



    def output_image(self, filename, show_solution=True, show_explored=False):
        from PIL import Image, ImageDraw


        cell_size = 50
        cell_border = 2

        # Create a blank canvas
        img = Image.new(
            "RGBA",
            (self.width * cell_size, self.height * cell_size),
            "black"
        )
        draw = ImageDraw.Draw(img)

        solution = self.solution[1] if self.solution is not None else None
        for i, row in enumerate(self.walls):
            for j, col in enumerate(row):

                # Walls
                if col:
                    fill = (40, 40, 40)

                # Start
                elif (i, j) == self.start:
                    fill = (255, 0, 0)

                # Goal
                elif (i, j) == self.goal:
                    fill = (0, 171, 28)

                # Solution
                elif solution is not None and show_solution and (i, j) in solution:
                    fill = (220, 235, 113)

                # Explored
                elif solution is not None and show_explored and (i, j) in self.explored:
                    fill = (212, 97, 85)

                # Empty cell
                else:
                    fill = (237, 240, 252)

                # Draw cell
                draw.rectangle(
                    ([(j * cell_size + cell_border, i * cell_size + cell_border),
                      ((j + 1) * cell_size - cell_border, (i + 1) * cell_size - cell_border)]),
                    fill=fill
                )

        img.save(filename)
        print(f"Image saved as {filename}")


class MazeGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Maze Solver")

        # Input for maze filename
        self.label = tk.Label(root, text="Enter Maze Filename:", font=('Arial', 11), bg="#ed7d31", fg='black')
        self.label.pack(pady=10, anchor=tk.W, padx=20)

        self.filename_entry = tk.Entry(root, width=30)
        self.filename_entry.pack(anchor=tk.W, padx=20)

        # Button to load maze
        self.load_button = tk.Button(root, text="Load Maze", bg="#fdf7bf", fg='black', command=self.load_maze)
        self.load_button.pack(pady=5, anchor=tk.W, padx=20)

        # Buttons for solving with different algorithms
        self.dfs_button = tk.Button(root, text="Solve with DFS", bg="#5b9bd5", fg='black', command=lambda: self.solve_maze("DFS"))
        self.dfs_button.pack(anchor=tk.W, padx=20)

        self.bfs_button = tk.Button(root, text="Solve with BFS", bg="#5b9bd5", fg='black', command=lambda: self.solve_maze("BFS"))
        self.bfs_button.pack(pady=5, anchor=tk.W, padx=20)

        self.heuristic_button = tk.Button(root, text="Solve with Heuristic BFS", bg="#aab4db", fg='black',
                                          command=lambda: self.solve_maze("Heuristic BFS"))
        self.heuristic_button.pack(anchor=tk.W, padx=20)

        self.heuristic_button = tk.Button(root, text="Solve with Heuristic A*", bg="#aab4db", fg='black',
                                          command=lambda: self.solve_maze("Heuristic A*"))
        self.heuristic_button.pack(anchor=tk.W, padx=20)

        self.optimal_button = tk.Button(root, text="Solve with Optimal algorithm", bg="#5b9bd5", fg='black',
                                          command=lambda: self.solve_maze("Optimal"))
        self.optimal_button.pack(pady=5, anchor=tk.W, padx=20)

        self.output_label = tk.Label(root, text="", bg='#ffd78e', fg='black')
        self.output_label.pack(anchor=tk.W, padx=20)

        # Label to display the maze image
        self.image_label = tk.Label(root, bg='#ffd78e')
        self.image_label.pack(anchor=tk.E, padx=20)

    def load_maze(self):
        filename = filedialog.askopenfilename(title="Select Maze File", filetypes=[("Text Files", "*.txt")])
        if not filename:
            return  # User cancelled the dialog
        print(f"Selected file: {filename}")  # Print the selected filename
        if not os.path.isfile(filename):
            messagebox.showerror("Error", "Maze file not found.")
            return
        self.filename_entry.delete(0, tk.END)
        self.filename_entry.insert(0, filename)
        self.maze = Maze(filename)
        self.output_label["text"] = "Maze Loaded!"

    def solve_maze(self, algorithm):
        if not hasattr(self, 'maze'):
            messagebox.showerror("Error", "Load a maze first!")
            return

        try:
            if algorithm == "DFS":
                self.maze.solve_dfs()
            elif algorithm == "BFS":
                self.maze.solve_bfs()
            elif algorithm == "Heuristic BFS":
                self.maze.HeuristicBFS()
            elif algorithm == 'Heuristic A*':
                self.maze.HeuristicA()
            else:
                self.maze.solve_Bellman_Ford()
                # self.maze.solve_Floyd_Warshall()
                # self.maze.solve_dijkstra()

            self.output_label["text"] = f"Solved with {algorithm}! States Explored: {self.maze.num_explored}"

            # Output the solved maze image
            image_filename = "maze_solution.png"
            self.maze.output_image(image_filename, show_explored=True)

            # Call the correct method to display the image
            self.display_image(image_filename)

        except Exception as e:
            messagebox.showerror("Error", str(e))

    def display_image(self, filename):
        if not os.path.isfile(filename):
            messagebox.showerror("Error", "Image file not found.")
            return

        try:
            # Load the image
            img = Image.open(filename)
            img = img.resize((400, 305))  # Resize for display
            img_tk = ImageTk.PhotoImage(img)


            # Update the label to display the image
            self.image_label.config(image=img_tk)
            self.image_label.image = img_tk  # Keep a reference to avoid garbage collection
        except Exception as e:
            messagebox.showerror("Error", f"Could not load image: {str(e)}")


# Main Application Loop
if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("800x600")
    root.configure(bg='#ffd78e')
    app = MazeGUI(root)
    root.mainloop()
