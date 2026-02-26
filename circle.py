import os
import pandas as pd
import numpy as np

def generate_circle_csv():
    # Table center relative to robot base
    center = [-0.1, -0.3]
    radius = 0.2
    num_points = 200

    # Generate points
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)

    # Dynamic path: Find workspace root from script location, create data/motion_safe.csv
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Folder containing circle.py
    workspace_root = script_dir  # Up one level to ros2-master
    output_dir = os.path.join(workspace_root, 'data')
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, 'motion_safe.csv')

    # Overwrite existing CSV with new data
    df = pd.DataFrame({'x': x, 'y': y})
    df.to_csv(csv_path, index=False)
    print(f"Success: Overwrote {num_points} points in {csv_path}")
    print(f"Workspace detected: {workspace_root}")

if __name__ == "__main__":
    generate_circle_csv()
