import tkinter as tk 
import keyboard
import time

#GUI CLASS
class GUI_MAP(): 
    def __init__(self): 
        self.matrix=[
                    [1, 1, 1, 1, 1, 1, 1, 1],
                    [1, 0, 0, 0, 0, 0, 3, 1],
                    [1, 0, 1, 1, 1, 0, 0, 1],
                    [1, 0, 0, 0, 1, 0, 0, 1],
                    [1, 1, 1, 0, 1, 0, 0, 1],
                    [1, 1, 1, 0, 1, 0, 0, 1],
                    [1, 1, 1, 0, 1, 0, 0, 1],
                    [1, 1, 1, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 1, 1, 0, 1],
                    [1, 0, 1, 0, 0, 0, 0, 1],
                    [1, 0, 1, 1, 0, 0, 0, 1],
                    [1, 0, 0, 0, 0, 1, 0, 1],
                    [1, 2, 0, 0, 0, 0, 0, 1],
                    [1, 1, 1, 1, 1, 1, 1, 1]
                ]
        
        self.color_map = {
            1: "black",  #obstacles
            0: "white",  #path
            2: "red",    #S
            3: "green"   #F
        }

        self.square_size = 50
        self.rows, self.cols = 14, 8
        self.width = self.cols * self.square_size
        self.height = self.rows * self.square_size

        self.move_map = []
        self.moves = {
            'a': "L",
            'w': "F",
            's': "S",
            'd': "R"
        }

        # Find start position (green square)
        for row in range(self.rows):
            for col in range(self.cols):
                if self.matrix[row][col] == 3:
                    self.player_x = col
                    self.player_y = row
                    break
        
        self.direction = 'left'  # Initial direction
        self.directions = ['up', 'right', 'down', 'left']

    def build_GUI(self):
        # Create the main window
        self.root = tk.Tk()
        self.root.title("Map")

        # Create a Canvas to draw the squares
        self.canvas = tk.Canvas(self.root, width=self.width, height=self.height)
        self.canvas.pack()

        # Draw the squares
        self.draw_map()
        
        # Draw the arrow
        self.arrow = self.canvas.create_polygon(self.get_arrow_coords(), fill="black")

        # Bind keyboard events
        keyboard.on_press(self.on_key_event)

        self.root.mainloop()
    
    def draw_map(self):
        for row in range(self.rows):
            for col in range(self.cols):
                x1 = col * self.square_size
                y1 = row * self.square_size
                x2 = x1 + self.square_size
                y2 = y1 + self.square_size

                # Get the fill color from the matrix
                fill_color = self.color_map.get(self.matrix[row][col], "white")

                self.canvas.create_rectangle(x1, y1, x2, y2, fill=fill_color, outline="black")
    
    def get_arrow_coords(self):
        x = self.player_x * self.square_size + self.square_size // 2
        y = self.player_y * self.square_size + self.square_size // 2
        size = 15
        
        if self.direction == 'up':
            return [x, y - size, x - size, y + size, x + size, y + size]
        elif self.direction == 'right':
            return [x + size, y, x - size, y - size, x - size, y + size]
        elif self.direction == 'down':
            return [x, y + size, x - size, y - size, x + size, y - size]
        elif self.direction == 'left':
            return [x - size, y, x + size, y - size, x + size, y + size]

    def on_key_event(self, event): 
        if event.name in self.moves:
            move = self.moves[event.name]
            self.move_map.append(move)
            
            if move == "L":
                self.direction = self.directions[(self.directions.index(self.direction) - 1) % 4]
            elif move == "R":
                self.direction = self.directions[(self.directions.index(self.direction) + 1) % 4]
            elif move == "F":
                self.move_forward()
            
            self.canvas.coords(self.arrow, self.get_arrow_coords())
    
    def move_forward(self):
        prev_x, prev_y = self.player_x, self.player_y
        
        if self.direction == 'up' and self.player_y > 0 and self.matrix[self.player_y - 1][self.player_x] != 1:
            self.player_y -= 1
        elif self.direction == 'down' and self.player_y < self.rows - 1 and self.matrix[self.player_y + 1][self.player_x] != 1:
            self.player_y += 1
        elif self.direction == 'left' and self.player_x > 0 and self.matrix[self.player_y][self.player_x - 1] != 1:
            self.player_x -= 1
        elif self.direction == 'right' and self.player_x < self.cols - 1 and self.matrix[self.player_y][self.player_x + 1] != 1:
            self.player_x += 1
        
        # Draw trail from previous position to new position
        self.canvas.create_line(prev_x * self.square_size + self.square_size // 2,
                                prev_y * self.square_size + self.square_size // 2,
                                self.player_x * self.square_size + self.square_size // 2,
                                self.player_y * self.square_size + self.square_size // 2,
                                fill='black', width=2)

if __name__ == "__main__":
    gui = GUI_MAP()
    gui.build_GUI()
