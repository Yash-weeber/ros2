import pandas as pd
import matplotlib.pyplot as plt
import os

# Define the path to your CSV file
csv_path = os.path.expanduser('~/ros2_ws/data/motion_safe.csv')

if not os.path.exists(csv_path):
    print(f"Error: File not found at {csv_path}")
else:
    # Load the trajectory data
    df = pd.read_csv(csv_path)

    # Create the plot
    plt.plot(df['x'], df['y'], marker='o', linestyle='-', color='b', label='Mop Path')

    # Labeling with LaTeX formatting
    plt.xlabel(r'$X$ Position ($m$)')
    plt.ylabel(r'$Y$ Position ($m$)')
    plt.title(r'UR5 Cleaning Trajectory Visualization')

    plt.legend()
    plt.grid(True)

    # Save the output visualization
    plt.savefig('cleaning_trajectory.png')
    print("Plot successfully saved as 'cleaning_trajectory.png'")