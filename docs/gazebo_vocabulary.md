**[⬅️ Back to README](../README.md)**

## **Training a Custom DBoW2 Vocabulary for ORB-SLAM3**

This document is a step-by-step guide to training a custom DBoW2 vocabulary for ORB-SLAM3 from images collected in any Gazebo simulation environment. A custom vocabulary can improve BoW candidate detection in simulation environments that differ significantly from the real-world datasets (KITTI, EuRoC) used to train the default `ORBvoc.txt`.

> **Note on vocabulary effectiveness:** A custom vocabulary improves loop closure candidate detection (Stage 1) but does not fix geometric verification (Stage 2, Sim3 RANSAC). In visually sparse Gazebo environments, Sim3 RANSAC may still return zero inliers regardless of vocabulary quality. See [loop_closure.md](loop_closure.md) for details on when this matters. Additionally, a domain-specific vocabulary trained on a low-diversity environment can reduce discriminability between keyframes — see Section 5 for guidance on when to use a custom vocabulary vs the default.

---

### **1. Prerequisites**

- ROS2 Humble + Gazebo Harmonic stack running
- ORB-SLAM3 built at `~/ORB_SLAM3/` (see [orb_slam3_installation.md](orb_slam3_installation.md))
- OpenCV installed (`sudo apt install -y libopencv-dev`)
- `image_view` ROS2 package: `sudo apt install -y ros-humble-image-view`

---

### **2. Collect Images from Your Simulation**

The vocabulary should be trained on images representative of the environments where SLAM will run. Fly multiple missions covering different areas and altitudes of your world.

**Step 1 — Create output directory:**
```bash
mkdir -p ~/vocab_images
```

**Step 2 — Launch your simulation stack** (Gazebo + SITL + transforms). No need to run ORB-SLAM3 itself for this step.

**Step 3 — Start image saver** (Terminal A):
```bash
ros2 run image_view image_saver --ros-args \
    -r image:=/camera/left/image_raw \
    -p filename_format:="$HOME/vocab_images/frame%04d.jpg"
```

**Step 4 — Fly your missions.** Run each mission normally. Stop image saving between passes with `Ctrl+C` and restart it — images will continue numbering from where they left off if you use a counter, or collect them to separate folders and merge.

**Step 5 — Verify image count:**
```bash
ls ~/vocab_images/ | wc -l
```

**How many images are needed?**

| Coverage | Approximate images | Quality |
|---|---|---|
| Single straight pass | ~500–1000 | Minimal — only one viewpoint direction |
| 2–3 passes, different altitudes | ~3000–5000 | Good for moderate environments |
| 3+ passes, different areas + altitudes | 6000+ | Best — recommended |

For this project, 6,976 images from 3 passes (±4m perimeter at 2.8m, ±7m perimeter at 2.8m, ±4m perimeter at 4.5m) were used.

---

### **3. Clone and Build DBoW2**

```bash
cd ~
git clone https://github.com/dorian3d/DBoW2.git
cd DBoW2
mkdir build && cd build
cmake ..
make -j$(nproc)
```

---

### **4. Add `saveToTextFile()` to TemplatedVocabulary.h**

The default DBoW2 export uses OpenCV `FileStorage` (`.yml.gz`), which is **prohibitively slow** for large vocabularies — 5+ hours for ~850K nodes. A direct text file writer completes in seconds and produces a file ORB-SLAM3 can load directly.

Open `~/DBoW2/include/DBoW2/TemplatedVocabulary.h` and add this method inside the class definition (before the closing `}`):

```cpp
void saveToTextFile(const std::string &filename) const {
    std::ofstream f(filename);
    f << m_k << " " << m_L << " " << (int)m_scoring << " " << (int)m_weighting << "\n";
    f << m_nodes.size() << "\n";
    for (size_t i = 1; i < m_nodes.size(); ++i) {
        const Node& node = m_nodes[i];
        f << node.parent << " " << (node.isLeaf() ? 1 : 0) << " ";
        f << node.weight << " ";
        for (int j = 0; j < node.descriptor.cols; ++j) {
            f << (int)node.descriptor.at<uint8_t>(0, j);
            if (j < node.descriptor.cols - 1) f << " ";
        }
        f << "\n";
    }
    f.close();
}
```

Also add `#include <fstream>` near the top of the file if not already present.

---

### **5. Write the Training Script**

Open `~/DBoW2/demo/demo.cpp` and replace its contents with the following. Adjust `NIMAGES` to match your actual image count, and replace `/home/<your-username>` with your actual home directory path.

```cpp
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "DBoW2.h"

using namespace std;
using namespace DBoW2;

// ── Set this to your actual image count ──────────────────────────────────────
#define NIMAGES 6976

void loadFeatures(vector<vector<cv::Mat>> &features) {
    features.clear();
    features.reserve(NIMAGES);
    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
    cout << "Extracting ORB features from " << NIMAGES << " images..." << endl;
    for (int i = 0; i < NIMAGES; i++) {
        stringstream ss;
        ss << "/home/<your-username>/vocab_images/frame"
           << setw(4) << setfill('0') << i << ".jpg";
        cv::Mat image = cv::imread(ss.str(), 0);
        if (image.empty()) { features.push_back({}); continue; }
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
        vector<cv::Mat> desc_vec;
        for (int r = 0; r < descriptors.rows; r++)
            desc_vec.push_back(descriptors.row(r));
        features.push_back(desc_vec);
        if (i % 500 == 0) cout << "  Processed " << i << "/" << NIMAGES << endl;
    }
    cout << "Feature extraction complete." << endl;
}

void createVocabulary(vector<vector<cv::Mat>> &features) {
    // k=10, L=6 gives ~750K words — good balance of size and discriminability
    // Increase k or L for more words (larger file, slower load)
    const int k = 10;
    const int L = 6;
    const WeightingType weight = TF_IDF;
    const ScoringType score = L1_NORM;
    OrbVocabulary voc(k, L, weight, score);
    cout << "Creating vocabulary (k=" << k << ", L=" << L << ")..." << endl;
    voc.create(features);
    cout << "Vocabulary created with " << voc.size() << " words." << endl;
    string outfile = "/home/<your-username>/DBoW2/demo/custom_voc.txt";
    voc.saveToTextFile(outfile);
    cout << "Saved to " << outfile << endl;
}

int main() {
    vector<vector<cv::Mat>> features;
    loadFeatures(features);
    createVocabulary(features);
    return 0;
}
```

---

### **6. Build and Run**

```bash
cd ~/DBoW2/build
make -j$(nproc)
cd ~/DBoW2/demo
../build/demo_bow
```

Training takes 5–15 minutes depending on image count and hardware. Progress is printed every 500 images.

Expected output:
```
Extracting ORB features from 6976 images...
  Processed 0/6976
  Processed 500/6976
  ...
Feature extraction complete.
Creating vocabulary (k=10, L=6)...
Vocabulary created with 747923 words.
Saved to /home/<your-username>/DBoW2/demo/custom_voc.txt
```

---

### **7. Copy to ORB-SLAM3 and Update Launch File**

**Copy the vocabulary:**
```bash
cp ~/DBoW2/demo/custom_voc.txt ~/ORB_SLAM3/Vocabulary/custom_voc.txt
```

Verify:
```bash
ls -lh ~/ORB_SLAM3/Vocabulary/custom_voc.txt
```
Expected: file size proportional to word count (~100MB for ~750K words).

**Update `vslam.launch.py`** to point to the new vocabulary:

> **Path note:** All commands in this document use `~/Desktop/multi_uav_slam` as the repository root as this is where the project was set up during development. If you have cloned the repository elsewhere, replace `~/Desktop/multi_uav_slam` with your actual repository root path in all commands.

In `~/Desktop/multi_uav_slam/ros2_ws/src/iris_vslam/launch/vslam.launch.py`, update the vocabulary path:

```python
vocab_path = os.path.join(os.path.expanduser('~'), 'ORB_SLAM3', 'Vocabulary', 'custom_voc.txt')
```

Rebuild:
```bash
cd ~/Desktop/multi_uav_slam/ros2_ws && colcon build --packages-select iris_vslam
```

To revert to the original vocabulary:
```python
vocab_path = os.path.join(os.path.expanduser('~'), 'ORB_SLAM3', 'Vocabulary', 'ORBvoc.txt')
```

---

### **8. When to Use a Custom Vocabulary vs Default**

| Scenario | Recommendation |
|---|---|
| Indoor environment with walls always close to UAV | Use default `ORBvoc.txt` — wall features are rich enough for BoW matching |
| Open environment with only scattered small features | Custom vocabulary may help Stage 1 (candidate detection) |
| Sim3 RANSAC returning 0 inliers regardless of vocabulary | Vocabulary is not the bottleneck — see [loop_closure.md](loop_closure.md) |
| Low-diversity environment (same textures everywhere) | Custom vocabulary may hurt discriminability — test both |

**Key finding from this project:** The default `ORBvoc.txt` combined with the original unmodified ORB-SLAM3 loop closure pipeline produced better ATE than all combinations using the custom Gazebo vocabulary. A domain-specific vocabulary trained on a low-diversity environment reduces inter-keyframe discriminability, causing false loop closure candidates when thresholds are relaxed. See [loop_closure.md](loop_closure.md) Section 2 for the full comparison.

---

### **9. Notes**

- **Always use `saveToTextFile()`** — OpenCV `FileStorage` export (`.yml.gz`) took 5+ hours for a 750K-word vocabulary. The direct text writer completes in seconds.
- **Images must be grayscale-compatible** — the training script reads images with `cv::imread(path, 0)` (grayscale). Color images are automatically converted.
- **Frame numbering must be continuous** — the script uses zero-padded sequential numbers (`frame0000.jpg`, `frame0001.jpg`, ...). If collecting images across multiple sessions, ensure no gaps in numbering.
- **More diverse images = better vocabulary** — vary altitude, direction, and area coverage across collection passes.
- **Vocabulary is environment-specific** — if the Gazebo world changes significantly (different textures, different layout), retrain with images from the new world.