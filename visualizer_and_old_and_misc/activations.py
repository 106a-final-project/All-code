import numpy as np

def detect_activation(grid_new, grid_avg, buff_size=20, threshold=0.15):
    """
    Detects activations in the grid based on differences with the running average.

    Parameters:
    grid_new: np.array of shape (36, 36) - the latest grid reading.
    grid_avg: np.array of shape (36, 36, buff_size) - the historical grid data.
    buff_size: int - the number of frames for averaging.
    threshold: float - the threshold for activation.

    Returns:
    activations: np.array of shape (36, 36) - detected activations.
    updated_grid_avg: np.array of shape (36, 36, buff_size) - updated grid history.
    """

    # Compute the average across the last `buff_size` frames (axis=2)
    avg_grid = np.sum(grid_avg, axis=2) / buff_size

    # Compute the difference between the current grid and the average grid
    grid_diff = grid_new - avg_grid

    # Find activations where difference exceeds threshold relative to average
    activations = np.where(np.abs(grid_diff) > threshold * np.abs(avg_grid), grid_diff, 0)

    # Shift grid_avg by one frame (remove last frame, insert new one at position 0)
    updated_grid_avg = np.roll(grid_avg, shift=1, axis=2)
    updated_grid_avg[:, :, 0] = grid_new

    return activations, updated_grid_avg

# Test the function with sample data
# grid_avg initially set with random values for testing
grid_avg = np.random.rand(36, 36, 20)
grid_new = np.random.rand(36, 36)

activations, updated_grid_avg = detect_activation(grid_new, grid_avg)

print("Activations:\n", activations)
print("\nUpdated Grid Average (First Layer):\n", updated_grid_avg[:, :, 0])
