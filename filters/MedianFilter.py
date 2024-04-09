class MedianFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []

    def update(self, value):
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)

    def get_median(self):
        sorted_values = sorted(self.values)
        window_midpoint = len(sorted_values) // 2
        if len(sorted_values) % 2 == 0:
            return (sorted_values[window_midpoint - 1] + sorted_values[window_midpoint]) / 2
        else:
            return sorted_values[window_midpoint]

    def reset(self, value):
        self.values = [value]
