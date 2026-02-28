import os
import matplotlib.pyplot as plt
import pandas as pd

def resample_trajectory(iter):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_csv = os.path.join(script_dir, "dmp_trajectory_feedback.csv")
    output_csv = os.path.join(script_dir, "data", 'motion_safe.csv')

    # 1. Read Original Data
    if not os.path.exists(input_csv):
        print(f"Error: {input_csv} not found!")
        return

    df = pd.read_csv(input_csv, dtype={'iter': str, 'step': str, 'timestamp': str})
    df = df[df['iter'] == iter].copy()
    df = df.iloc[::20]

    if df.empty:
        print("CSV is empty!")
        return

    df['x'] = df['x'].astype(float)
    df['y'] = df['y'].astype(float)

    # 2. Analyze Original Shape
    min_x, max_x = df['x'].min(), df['x'].max()
    min_y, max_y = df['y'].min(), df['y'].max()

    orig_width = max_x - min_x if max_x != min_x else 1.0
    orig_height = max_y - min_y if max_y != min_y else 1.0

    # 3. Define requested boundaries
    SAFE_MIN_X = -0.3
    SAFE_MAX_X = 0.3
    SAFE_MIN_Y = -0.8
    SAFE_MAX_Y = -0.41

    x_low, x_high = min(SAFE_MIN_X, SAFE_MAX_X), max(SAFE_MIN_X, SAFE_MAX_X)
    y_low, y_high = min(SAFE_MIN_Y, SAFE_MAX_Y), max(SAFE_MIN_Y, SAFE_MAX_Y)

    safe_width = x_high - x_low
    safe_height = y_high - y_low

    # Calculate scaling factor to preserve aspect ratio
    scale_w = safe_width / orig_width if orig_width > 0 else 1.0
    scale_h = safe_height / orig_height if orig_height > 0 else 1.0
    scale = min(scale_w, scale_h)

    # 4. Apply Scaling, Translation and Clamping
    print(f"Original X Range: {min_x:.2f} to {max_x:.2f}")
    print(f"Original Y Range: {min_y:.2f} to {max_y:.2f}")
    print(f"Scaling all points by factor: {scale:.4f}")

    df['x'] = ((df['x'] - min_x) * scale_w + x_low).clip(x_low, x_high)
    df['y'] = ((df['y'] - min_y) * scale_h + y_low).clip(y_low, y_high)

    # 5. Save Output
    df[['iter', 'step', 'x', 'y', 'timestamp']].to_csv(
        output_csv, index=False, float_format='%.5f'
    )

    # Final safety check
    out_of_bounds = df[~(df['x'].between(x_low, x_high) & df['y'].between(y_low, y_high))]
    if not out_of_bounds.empty:
        print(f"[ERROR] Found {len(out_of_bounds)} points outside bounds after clamping.")
    else:
        print("[OK] All resampled points are within safe bounds.")

    print(f"\n[SUCCESS] Saved safe trajectory to: {output_csv}")
    print(f"New X Range: [{x_low}, {x_high}]")
    print(f"New Y Range: [{y_low}, {y_high}]")

    # 6. Plot
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.scatter(df['x'].values, df['y'].values, label="Original Trajectory")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Original Trajectory")
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.scatter(df['x'].values, df['y'].values, label="Resampled Trajectory")
    plt.xlabel("X Position (Safe)")
    plt.ylabel("Y Position (Safe)")
    plt.title("Resampled Safe Trajectory")
    plt.grid(True)

    plt.tight_layout()
    output_plot = os.path.join(script_dir, "data", "trajectory_comparison.png")
    plt.savefig(output_plot)
    plt.show()
    print(f"Saved trajectory comparison plot to: {output_plot}")

if __name__ == '__main__':
    resample_trajectory("100")