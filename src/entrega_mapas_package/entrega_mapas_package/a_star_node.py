#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import heapq
import os
from PIL import Image

from .planning import ThetaStar

class AStarPlanner(ThetaStar, Node):
    def __init__(self):
        super().__init__('a_star_node')
        self.get_logger().info('Theta-style Planner Initialized')

    def load_map(self, map_path):
        """Loads a map image and converts it to a binary grid (0: free, 1: obstacle)."""
        try:
            img = Image.open(map_path).convert('L') # Grayscale
            # Resize for performance if needed, or keep original resolution
            # img = img.resize((100, 100)) 
            grid_data = np.array(img)

            # Simple thresholding: Dark pixels are obstacles
            # Assuming 255 is free space (white) and 0 is obstacle (black)
            # You might need to invert this depending on your map
            binary_grid = (grid_data <= 128).astype(int) 
            return binary_grid
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            return None







    def plot_path(self, grid, path, start, goal, count, title="Theta-style Path"):
        plt.figure(figsize=(6, 6))
        plt.imshow(grid, cmap='Greys', origin='upper') # Display grid (0:White, 1:Black)

        if path:
            # Extract Y and X coordinates (row, col) -> (y, x) for plotting? 
            # imshow uses (row, col) but plot uses (x, y) usually.
            # Matplotlib imshow: axis 0 is y (rows), axis 1 is x (cols).
            ys = [p[0] for p in path]
            xs = [p[1] for p in path]
            plt.plot(xs, ys, color='red', linewidth=2, label='Path')

        plt.scatter(start[1], start[0], color='green', s=100, marker='o', label='Start') # x=col, y=row
        plt.scatter(goal[1], goal[0], color='blue', s=100, marker='x', label='Goal')

        plt.title(f"{title} (Theta-style)\nNodes: {count}")
        plt.legend()
        plt.grid(False) # Grid lines might be confusing on top of pixel grid

        # Save plot to a file instead of showing (good for docker/headless) 
        output_file = os.path.join("maps", f"astar_result_{title.replace(' ', '_')}.png")
        plt.savefig(output_file)
        print(f"Plot saved to {os.path.abspath(output_file)}")
        # plt.show() 

    def save_debug_map(self, grid, start, goal, title="Debug Map"):
            """Guarda el mapa con una rejilla y los puntos de inicio/fin marcados."""
            # Configuración de la figura
            plt.figure(figsize=(12, 12)) # Hacemos la imagen grande para ver bien
            plt.imshow(grid, cmap='Greys', origin='upper')

            # Marcar inicio (Verde) y fin (Rojo)
            # IMPORTANTE: En plot es (Columna, Fila) -> (Y, X) del array
            plt.scatter(start[1], start[0], color='green', s=150, label=f'Start {start}', edgecolors='black') 
            plt.scatter(goal[1], goal[0], color='red', s=150, marker='X', label=f'Goal {goal}', edgecolors='black')

            # Activar rejilla y ejes
            plt.grid(True, which='both', color='red', linestyle='--', linewidth=0.5, alpha=0.3)
            plt.minorticks_on() 

            # Títulos y ejes
            plt.title(f"{title}\nEje X (Abajo) = Columnas | Eje Y (Izquierda) = Filas\nBusca coordenadas blancas para Start/Goal")
            plt.xlabel("Columna (X)")
            plt.ylabel("Fila (Y)")
            plt.legend()

            # Guardar
            output_file = os.path.join("maps", f"debug_coords_{title.replace(' ', '_')}.png")
            plt.savefig(output_file, dpi=150) # dpi alto para ver mejor los números
            self.get_logger().info(f"--> MAPA DE COORDENADAS GUARDADO EN: {output_file}")
            plt.close()

def create_dummy_map(filename, size=(50, 50)):
    # Create a white background
    img = Image.new('L', size, 255)
    pixels = img.load()

    # Draw some walls (black)
    for i in range(10, 40):
        pixels[20, i] = 0 # Vertical wall
    for i in range(20, 40):
        pixels[i, 10] = 0 # Horizontal wall

    img.save(filename)
    print(f"Created map: {filename}")

def main(args=None):
    rclpy.init(args=args)
    planner = AStarPlanner()

    # Define scenarios
    scenarios = [
        # (Name, MapFilename, Start, Goal)
        #("Scenario 1 Practice Map", "practica2.png", (150, 200), (350, 700)), # Dynamic goal based on size
        ("Scenario 2 Maze01", "Maze01.png", (15, 5), (80, 150)),
        ("Scenario 3 Maze02", "Maze02.png", (250, 250), (500, 250)),
        ("Scenario 4 Casa", "casa.png", (50, 100), (800, 1100)),
        ("Scenario 5 Prado", "prado.png", (50, 30), (100, 400))
    ]

    create_dummy_map("map_simple.png")
    # Create trapped map
    img2 = Image.new('L', (20, 20), 255)
    pixels = img2.load()
    for i in range(20):
        pixels[i, 10] = 0
    img2.save("map_trapped.png")

    for name, filename, start, goal in scenarios:
        planner.get_logger().info(f"Running {name}...")
        map_path = os.path.join("maps", filename) if filename not in ["map_simple.png", "map_trapped.png"] else filename

        # Quick hack for simple/trapped to be found if not in maps/ (though we save them in cur dir)
        # Actually my previous code saved simple/trapped in CWD, but referenced them directly.
        # User asked to check images in maps/, let's assume external images are in maps/

        grid = planner.load_map(map_path)
        if grid is None:
             # Try simple/trapped in current dir if failed
             grid = planner.load_map(filename)

        if grid is not None:
             rows, cols = grid.shape

             # Handle dynamic coordinates
             current_goal = goal
             if current_goal == (None, None):
                 current_goal = (rows - 20, cols - 20)

             # Validate coordinates
             s = start
             g = current_goal

             # Ensure within bounds
             s = (max(0, min(s[0], rows-1)), max(0, min(s[1], cols-1)))
             g = (max(0, min(g[0], rows-1)), max(0, min(g[1], cols-1)))

             # planner.save_debug_map(grid, s, g, title=name)
             planner.get_logger().info(f"Planning {name} from {s} to {g}")
             path, count = planner.plan(grid, s, g)
             if path:
                 planner.plot_path(grid, path, s, g, count, title=name)
             else:
                 planner.get_logger().warn(f"No path found for {name}. Start/Goal might be in obstacle.")



    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
