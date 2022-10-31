"""
From https://www.youtube.com/watch?v=LqPPvPKUfV4&list=PL1P11yPQAo7opIg8r-4BMfh1Z_dCOfI0y
"""

import glfw

# Initialize glfw library
if not glfw.init():
    raise Exception("glfw cannot be initialized!")

# Create a window
window = glfw.create_window(1280, 720, "My OpenGL window", None, None) # last two arguments are monitor (None for windowed mode) and share resouces

# Check if whindow was created
if not window:
    glfw.terminate()
    raise Exception("glfw window cannot be created!")

# Set window's position
glfw.set_window_pos(window, 400, 200)

# Make the context current
glfw.make_context_current(window)

# The main application loop
while not glfw.window_should_close(window):
    glfw.poll_events()
    glfw.swap_buffers(window)

# Terminate glfw, free up allocated resources
glfw.terminate()