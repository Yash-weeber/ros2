import matplotlib.pyplot as plt
import pandas as pd
import os

def plot_trajectory(iter):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_csv = os.path.join(script_dir, "data", "motion_safe.csv")

    if not os.path.exists(input_csv):
        print(f"Error: {input_csv} not found!")
        return

    df = pd.read_csv(input_csv, dtype={'iter': str, 'step': str, 'timestamp': str})
    df = df[df['iter'] == iter]

    if df.empty:
        print("CSV is empty!")
        return

    df['x'] = df['x'].astype(float)
    df['y'] = df['y'].astype(float)

    plt.figure(figsize=(8, 6))
    plt.plot(df['x'], df['y'], marker='o', linestyle='-', color='blue')
    plt.title(f'Trajectory for Iteration {iter}')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid()
    plt.axis('equal')
    plt.xlim(-0.5, 0.5)
    plt.ylim(-1.0, 0.0)
    plt.show()
    
if __name__ == "__main__":
    plot_trajectory(iter='-2')