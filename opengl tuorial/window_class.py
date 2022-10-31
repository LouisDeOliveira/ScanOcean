"""
From https://www.youtube.com/watch?v=LqPPvPKUfV4&list=PL1P11yPQAo7opIg8r-4BMfh1Z_dCOfI0y
"""

import glfw

class Window:
    def __init__(self, width:int, height:int, title:str):
        if not glfw.init():
            raise Exception("glfw cannot be initialized!")

        self._win = glfw.create_window(width, height, title, None, None)

        if not self._win:
            glfw.terminate()
            raise Exception("glfw window cannot be created!")

        glfw.set_window_pos(self._win, 400, 200)
        glfw.make_context_current(self._win)

    def main_loop(self):
        while not glfw.window_should_close(self._win):
            glfw.poll_events()
            glfw.swap_buffers(self._win)
        
        glfw.terminate()

if __name__ == "__main__":
    win = Window(1280, 720, "My OpenGL window")
    win.main_loop()
