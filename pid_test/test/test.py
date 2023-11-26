import pandas as pd
import matplotlib.pyplot as plt

# Function to parse a line
def parse_line(line):
    parts = line.split(',')
    return {part.split(':')[0].strip(): float(part.split(':')[1].strip()) for part in parts}

# Load and parse data
with open('data.txt', 'r') as f:
    data = [parse_line(line) for line in f.readlines()]

# Convert to DataFrame
df = pd.DataFrame(data)
df['velocity_derivative'] = df['present_vel'].diff()

# Plot data
ax = df.plot()
ylim = 75
ax.set_ylim(-ylim, ylim)
ax.set_xlim(0, 500)
plt.show()