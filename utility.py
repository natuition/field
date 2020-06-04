class Logger:
    """
    Writes into the file with specified name str data, flushing data on each receiving
    """

    def __init__(self, file_name):
        self._file = open(file_name, "w")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._file.__exit__(exc_type, exc_val, exc_tb)

    def write_and_flush(self, s):
        self._file.write(s)
        self._file.flush()

    def write(self, s):
        self._file.write(s)

    def close(self):
        self._file.close()
