from tkinter import Tk, Label, Button, Text, Checkbutton, Frame

class TkinterUtils:
    @staticmethod
    def initialize_tkinter(title: str) -> Tk:
        root = Tk()
        root.title(title)
        root.resizable(False, False)
        return root
    
    @staticmethod
    def create_check_box(text, function, side, padx, pady, master):
        check_box = Checkbutton(master, text=text, command=function)
        check_box.pack(side=side, padx=padx, pady=pady)
        return check_box
    
    @staticmethod
    def create_text(master, height, width, side, padx, pady):
        inputtxt = Text(master, height=height, width=width)
        inputtxt.pack(side=side, padx=padx, pady=pady)
        return inputtxt
    
    @staticmethod
    def create_button(text, function, side, padx, pady, master):
        button = Button(master, text=text, command=function)
        button.pack(side=side, padx=padx, pady=pady)
        return button
    
    @staticmethod
    def create_label(text, side, padx, pady, master, fg_color="black", wraplength=300):
        label = Label(master, text=text, fg=fg_color, wraplength=wraplength, justify="left")
        label.pack(side=side, padx=padx, pady=pady)
        return label
    
    @staticmethod
    def create_frame(master, side):
        frame = Frame(master)
        frame.pack(side=side, fill="x")
        return frame