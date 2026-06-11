import csv
import os


class csvWriter:

    def __init__(self, filename: str):
        """Creates and opens a new CSV file.

        If the file already exists, it will overwrite it.
        """
        self.filename = filename
        # 'newline=""' is recommended by the csv module documentation to prevent double-spacing
        self.file = open(self.filename, mode="w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        print(f"File '{self.filename}' created and opened successfully.")

    def append(self, row_data: list):
        """Inserts a new row to the end of the file.

        'row_data' indicates the values per column.
        """
        if self.file.closed:
            raise ValueError("Cannot append data. The file is already closed.")

        self.writer.writerow(row_data)

    def close(self):
        """Closes the CSV file safely."""
        if not self.file.closed:
            self.file.close()
            print(f"File '{self.filename}' closed successfully.")
        else:
            print(f"File '{self.filename}' was already closed.")

    def __del__(self):
        """Destructor to ensure the file closes if the object is deleted

        or the script exits unexpectedly.
        """
        try:
            self.close()
        except Exception:
            pass