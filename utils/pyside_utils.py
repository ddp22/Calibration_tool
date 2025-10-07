from PySide6.QtWidgets import QPushButton

class PySideUtils:
    def __init__(self, button_size=(150, 35)):
        self.button_size = button_size

    @staticmethod
    def create_button(text: str, parent, position: tuple, callback: callable, size: tuple = (150, 35)) -> QPushButton:
        button = QPushButton(text, parent)
        button.resize(*size)
        button.move(*position)
        button.clicked.connect(callback)
        button.show()
        return button