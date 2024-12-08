class CommandError(Exception):
    """An error with a message to provide to the user"""

    def __init__(self, message: str):
        super().__init__(message)
        self.message = message
