import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


runs_path = Path("./output/tests/area_1_parameters/height/")



# histograms: list of np.histogram results
# assume each has same bins
H = np.stack([h[0] / np.sum(h[0]) for h in histograms])  # normalize
bins = histograms[0][1]

plt.figure(figsize=(8, 6))
plt.imshow(H, aspect='auto', cmap='viridis', 
           extent=[bins[0], bins[-1], 0, len(H)])
plt.colorbar(label='Relative frequency')
plt.xlabel("Z (height)")
plt.ylabel("Experiment index")
plt.title("Distribution of nodes along Z axis across experiments")
plt.show()
