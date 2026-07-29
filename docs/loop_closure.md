**[⬅️ Back to README](../README.md)**

## **Loop Closure Investigation — Experimental Reference**

> **This document describes experimental modifications made to ORB-SLAM3's loop closure pipeline during the single-UAV SLAM evaluation phase. These changes were ultimately not adopted in the final system.** The original unmodified ORB-SLAM3 pipeline with the default `ORBvoc.txt` vocabulary was found to perform better overall (see Section 7). This document is retained as a reference for anyone who wants to understand what was tried, why it was tried, and why it was reverted.

---

### **1. Background — Why Loop Closure Fails in Gazebo**

ORB-SLAM3's loop closure pipeline has two stages:

1. **Candidate detection** — "Have I been here before?" Uses DBoW2 (Bag of Words) to find keyframes whose visual appearance matches the current frame.
2. **Geometric verification** — "Prove these two views are the same place." Uses Sim3 RANSAC to find a geometrically consistent transformation between matched map point pairs.

Both stages failed in Gazebo for the same fundamental reason: the default `ORBvoc.txt` vocabulary was trained on real-world images. Gazebo's plain walls, simple floor textures, and poster images produce ORB descriptors that map to very few or inconsistent visual words in this vocabulary.

**Root cause progression confirmed via debug prints:**

```
sharingWords = 0  (early mission)    →  No BoW candidates             ✗
sharingWords = 5–34 (return leg)     →  Candidates found              ✓
numBoWMatches = 9–25                 →  Passes threshold               ✓
nInliers = 0  (always)              →  Sim3 RANSAC fails              ✗
```

Even after custom vocabulary training (Approach A), `nInliers=0` persisted — Gazebo's features are geometrically inconsistent regardless of vocabulary quality.

---

### **2. Why These Changes Were Reverted**

Before going into the details, it is important to understand why the final system uses the original unmodified pipeline. After a controlled 4-configuration ATE evaluation on the `i20_loop_16x16` mission, the results were:

| Configuration | RMSE | Mean | Max error |
|---|---|---|---|
| Modified LC + Gazebo vocab | 0.601 m | 0.521 m | 1.147 m |
| Modified LC + ORB vocab | 0.111 m | 0.072 m | **1.326 m** |
| Original LC + Gazebo vocab | 0.159 m | 0.135 m | 0.314 m |
| **Original LC + ORB vocab** | **0.155 m** | **0.136 m** | **0.292 m** |

Key findings:

- **Modified LC + Gazebo vocab** produces catastrophic results — the lowered thresholds accept false loop closure candidates from the over-fitted vocabulary, actively corrupting the map.
- **Modified LC + ORB vocab** has the best RMSE but the worst max error (1.33m) — a single bad correction event that would be unacceptable in a multi-UAV shared map.
- **Original LC + ORB vocab** is the most stable configuration across all metrics, with the lowest worst-case error.
- The original ORB-SLAM3 pipeline fires loop closure reliably on its own when the loop is large enough and the environment has sufficient wall features (see Section 7 for details).

---

### **3. Approach A — Custom DBoW2 Vocabulary (Partial Fix)**

A new vocabulary was trained specifically on images from the Gazebo simulation. This fixed Stage 1 (candidates are now found on the return leg) but not Stage 2 (Sim3 RANSAC still returns 0 inliers).

**Why it doesn't help overall:** The gazebo vocabulary is over-fitted to the low-diversity indoor environment. Many keyframes share the same visual words, reducing discriminability. When thresholds are relaxed (see Section 4), this causes false positives. When thresholds are kept strict, it behaves similarly to ORBvoc.txt.

For vocabulary training steps, see [Gazebo Vocabulary Training](gazebo_vocabulary.md).

---

### **4. Approach B — Spatial Fallback (Investigated, Not Adopted)**

When Sim3 RANSAC fails, a spatial fallback was added to bypass geometric verification entirely and use the SLAM pose estimate directly to build the loop closure transformation.

**Why it was tried:** Tracking state remained 2 (OK) throughout all missions — the pose estimates are reliable enough that spatial proximity is a trustworthy loop closure signal even when appearance-based verification fails.

**Why it was reverted:** The combination of lowered thresholds + spatial fallback introduces false loop closures that degrade ATE. The original pipeline is more conservative but more reliable. The spatial fallback is only warranted in environments where the standard pipeline produces zero loop closures at all — in this project's environments (`indoor20x20`, `indoor50x50`, `straight_path_with_pillars`), the original pipeline fires reliably on its own for large enough loops.

---

### **5. All Changes Made (For Reference)**

#### **5.1 `~/ORB_SLAM3/src/KeyFrameDatabase.cc`**

**Change:** `minCommonWords` threshold `0.8f` → `0.3f` (6 locations)

**Reason:** Controls the minimum number of shared BoW words for a keyframe to be considered a loop candidate. At `0.8`, Gazebo's sparse features meant zero candidates were ever surfaced. At `0.3`, candidates appear on the return leg of the mission.

**Apply:**
```bash
sed -i 's/minCommonWords = maxCommonWords\*0\.8f/minCommonWords = maxCommonWords*0.3f/g' \
    ~/ORB_SLAM3/src/KeyFrameDatabase.cc
```

**Undo:**
```bash
sed -i 's/minCommonWords = maxCommonWords\*0\.3f/minCommonWords = maxCommonWords*0.8f/g' \
    ~/ORB_SLAM3/src/KeyFrameDatabase.cc
```

---

#### **5.2 `~/ORB_SLAM3/src/LoopClosing.cc` — Change 1: Consistency threshold**

**Change:** `mnCovisibilityConsistencyTh = 3` → `1` (constructor, line ~41)

**Reason:** Number of consecutive keyframe confirmations required before `mbLoopDetected` is set true. At `3`, loop closure needs 3 consecutive confirmations — too strict for a short mission where the UAV passes the origin area only briefly. At `1`, a single confirmation suffices.

**Apply:**
```bash
sed -i 's/mnCovisibilityConsistencyTh = 3/mnCovisibilityConsistencyTh = 1/' \
    ~/ORB_SLAM3/src/LoopClosing.cc
```

**Undo:**
```bash
sed -i 's/mnCovisibilityConsistencyTh = 1/mnCovisibilityConsistencyTh = 3/' \
    ~/ORB_SLAM3/src/LoopClosing.cc
```

---

#### **5.3 `~/ORB_SLAM3/src/LoopClosing.cc` — Change 2: Hardcoded coincidence check**

**Change:** `mbLoopDetected = mnLoopNumCoincidences >= 3` → `>= mnCovisibilityConsistencyTh`

**Location:** `NewDetectCommonRegions()`, inside `if(mnLoopNumCoincidences > 0)` block

**Reason:** The original code had the threshold hardcoded as `>= 3`, completely ignoring the `mnCovisibilityConsistencyTh` member variable set in Change 1. This change makes detection actually respect the configured threshold.

**Apply:**
```bash
sed -i 's/mbLoopDetected = mnLoopNumCoincidences >= 3/mbLoopDetected = mnLoopNumCoincidences >= mnCovisibilityConsistencyTh/' \
    ~/ORB_SLAM3/src/LoopClosing.cc
```

**Undo:**
```bash
sed -i 's/mbLoopDetected = mnLoopNumCoincidences >= mnCovisibilityConsistencyTh/mbLoopDetected = mnLoopNumCoincidences >= 3/' \
    ~/ORB_SLAM3/src/LoopClosing.cc
```

---

#### **5.4 `~/ORB_SLAM3/src/LoopClosing.cc` — Change 3: Detection thresholds**

**Location:** Top of `DetectCommonRegionsFromBoW()` function

**Reason:** Original thresholds were calibrated for real-world environments with rich visual features. Gazebo produces only 5–19 BoW matches — below the original `nBoWMatches=20` — so the spatial fallback was never reached.

| Variable | Original | Changed | Reason |
|----------|----------|---------|--------|
| `nBoWMatches` | 20 | 5 | Gazebo features produce 5–19 BoW matches |
| `nBoWInliers` | 15 | 5 | Sim3 solver minimum inlier count |
| `nSim3Inliers` | 20 | 10 | Post-optimization inlier threshold |
| `nProjMatches` | 50 | 20 | Coarse projection match threshold |
| `nProjOptMatches` | 80 | 40 | Fine projection match threshold |

**Apply:** Manually edit the 5 values at the top of `DetectCommonRegionsFromBoW` in `LoopClosing.cc`:
```cpp
int nBoWMatches = 5;
int nBoWInliers = 5;
int nSim3Inliers = 10;
int nProjMatches = 20;
int nProjOptMatches = 40;
```

**Undo:** Restore original values:
```cpp
int nBoWMatches = 20;
int nBoWInliers = 15;
int nSim3Inliers = 20;
int nProjMatches = 50;
int nProjOptMatches = 80;
```

---

#### **5.5 `~/ORB_SLAM3/src/LoopClosing.cc` — Change 4: Spatial fallback block**

**Location:** Inside `DetectCommonRegionsFromBoW()`, immediately after the `while(!bConverge && !bNoMore)` Sim3 RANSAC loop, before the existing `if(bConverge)` block.

**Reason:** Sim3 RANSAC always returns `nInliers=0` in Gazebo because matched ORB feature pairs are geometrically inconsistent — the environment lacks enough visual distinctiveness for RANSAC to converge.

**How the fallback works:**
1. When `bConverge=false` (Sim3 RANSAC failed), enter the spatial fallback
2. Get all keyframes in the map
3. Filter to keyframes within `spatialRadius=9.0m` of the current pose
4. For each spatial candidate, count BoW matches — pick the best candidate
5. If best candidate has ≥ 3 BoW matches, build Sim3 directly from relative pose: `Tcm = Tcw_curr × Tcw_spatial⁻¹`
6. Run `SearchByProjection` with 20px search window — if ≥ 2 projection matches found, accept as loop candidate without calling `OptimizeSim3`

**Why `OptimizeSim3` is skipped:** It also returns 0 inliers for the same reason as Sim3 RANSAC — Gazebo features are too sparse for optimization to converge.

**Key parameters:**
```cpp
const float spatialRadius = 9.0f;  // covers ±4m mission + ~1.5x scale drift
int searchWindow = 20;             // wider than default 8px
int minProjMatches = 2;            // minimum projection matches to accept
int minBoWMatches = 3;             // minimum BoW matches to try a candidate
nBestNumCoindicendes = 3;          // forces mbLoopDetected = true immediately
```

**Code to insert** (between `while(!bConverge && !bNoMore)` loop and `if(bConverge)` block):

```cpp
// [SPATIAL FALLBACK] If Sim3 RANSAC failed, try pose-based initialization
cout << "[SPATIAL ENTRY] bConverge=" << bConverge << endl;
if(!bConverge)
{
    Sophus::SE3f Tcw_curr = mpCurrentKF->GetPose();
    Sophus::SE3f Twc_curr = mpCurrentKF->GetPoseInverse();
    Eigen::Vector3f t_curr = Twc_curr.translation();

    const float spatialRadius = 9.0f;

    std::vector<KeyFrame*> vpAllKFs = mpCurrentKF->GetMap()->GetAllKeyFrames();
    KeyFrame* pSpatialKF = nullptr;
    int nBestSpatialMatches = 0;

    for(KeyFrame* pKFsp : vpAllKFs)
    {
        if(!pKFsp || pKFsp->isBad()) continue;
        if(pKFsp->mnId >= mpCurrentKF->mnId - 20) continue; // skip recent KFs
        if(spConnectedKeyFrames.find(pKFsp) != spConnectedKeyFrames.end()) continue;

        Eigen::Vector3f t_sp = pKFsp->GetPoseInverse().translation();
        float dist = (t_curr - t_sp).norm();
        if(dist < spatialRadius)
        {
            std::vector<MapPoint*> vMatchedMPs_sp;
            int nMatches = matcherBoW.SearchByBoW(mpCurrentKF, pKFsp, vMatchedMPs_sp);
            cout << "[SPATIAL MATCH] KF" << pKFsp->mnId << " bowMatches=" << nMatches << endl;
            if(nMatches > nBestSpatialMatches)
            {
                nBestSpatialMatches = nMatches;
                pSpatialKF = pKFsp;
            }
        }
    }

    if(pSpatialKF && nBestSpatialMatches >= 3)
    {
        cout << "[SPATIAL LC] Spatial candidate KF" << pSpatialKF->mnId
             << " dist=" << (pSpatialKF->GetPoseInverse().translation() - t_curr).norm()
             << " matches=" << nBestSpatialMatches << endl;

        // Build Sim3 from relative pose (scale=1, stereo)
        Sophus::SE3f Tcw_sp = pSpatialKF->GetPose();
        Sophus::SE3f Tcm_se3 = Tcw_curr * Tcw_sp.inverse();
        g2o::Sim3 gScm_spatial(
            Tcm_se3.unit_quaternion().cast<double>(),
            Tcm_se3.translation().cast<double>(),
            1.0);
        g2o::Sim3 gSmw_spatial(
            pSpatialKF->GetRotation().cast<double>(),
            pSpatialKF->GetTranslation().cast<double>(),
            1.0);
        g2o::Sim3 gScw_spatial = gScm_spatial * gSmw_spatial;
        Sophus::Sim3f mScw_spatial = Converter::toSophus(gScw_spatial);

        // Collect map points from spatial candidate + covisibles
        std::vector<KeyFrame*> vpCovSpatial = pSpatialKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
        vpCovSpatial.push_back(pSpatialKF);
        std::set<MapPoint*> spMPsSpatial;
        std::vector<MapPoint*> vpMapPointsSpatial;
        std::vector<KeyFrame*> vpKFsSpatial;
        for(KeyFrame* pKFcov : vpCovSpatial)
        {
            for(MapPoint* pMPcov : pKFcov->GetMapPointMatches())
            {
                if(!pMPcov || pMPcov->isBad()) continue;
                if(spMPsSpatial.find(pMPcov) == spMPsSpatial.end())
                {
                    spMPsSpatial.insert(pMPcov);
                    vpMapPointsSpatial.push_back(pMPcov);
                    vpKFsSpatial.push_back(pKFcov);
                }
            }
        }

        // Project and count matches
        std::vector<MapPoint*> vpMatchedMP_spatial;
        vpMatchedMP_spatial.resize(mpCurrentKF->GetMapPointMatches().size(), nullptr);
        std::vector<KeyFrame*> vpMatchedKF_spatial;
        vpMatchedKF_spatial.resize(mpCurrentKF->GetMapPointMatches().size(), nullptr);
        int numProjSpatial = matcher.SearchByProjection(
            mpCurrentKF, mScw_spatial, vpMapPointsSpatial, vpKFsSpatial,
            vpMatchedMP_spatial, vpMatchedKF_spatial, 20, 1.5);

        cout << "[SPATIAL LC] Projection matches=" << numProjSpatial << endl;

        // Skip OptimizeSim3 — use pose-based Sim3 directly (Gazebo features too sparse)
        if(numProjSpatial >= 2)
        {
            if(nBestMatchesReproj < numProjSpatial)
            {
                nBestMatchesReproj = numProjSpatial;
                nBestNumCoindicendes = 3;
                pBestMatchedKF = pSpatialKF;
                g2oBestScw = gScw_spatial;
                vpBestMapPoints = vpMapPointsSpatial;
                vpBestMatchedMapPoints = vpMatchedMP_spatial;
                cout << "[SPATIAL LC] Accepted! KF" << pSpatialKF->mnId
                     << " projMatches=" << numProjSpatial << endl;
            }
        }
    }
}
// [END SPATIAL FALLBACK]
```

**Undo:** Remove the entire block between `// [SPATIAL FALLBACK]` and `// [END SPATIAL FALLBACK]` comments.

---

#### **5.6 Debug print statements**

Several `cout` debug prints were added during investigation.

| Print tag | Location | Purpose |
|-----------|----------|---------|
| `[LC BOW]` | Before `if(numBoWMatches >= nBoWMatches)` | Shows BoW match count vs threshold |
| `[SPATIAL ENTRY]` | Before `if(!bConverge)` | Confirms fallback is entered |
| `[SPATIAL MATCH]` | After `SearchByBoW` per candidate | Shows BoW matches per spatial candidate |
| `[SPATIAL LC]` | Key decision points | Tracks projection matches and acceptance |

---

### **6. Rebuild After Changes**

> **Path note:** All commands in this document use `~/Desktop/multi_uav_slam` as the repository root — this is where the project was set up during development. If you have cloned the repository elsewhere, replace `~/Desktop/multi_uav_slam` with your actual repository root path in all commands.

After applying any changes to ORB-SLAM3 source files:

```bash
cd ~/ORB_SLAM3 && ./build.sh
cd ~/Desktop/multi_uav_slam/ros2_ws && colcon build --packages-select iris_vslam
```

To fully revert all changes back to original:

```bash
# KeyFrameDatabase.cc
sed -i 's/minCommonWords = maxCommonWords\*0\.3f/minCommonWords = maxCommonWords*0.8f/g' \
    ~/ORB_SLAM3/src/KeyFrameDatabase.cc

# LoopClosing.cc — threshold and consistency changes
sed -i 's/mnCovisibilityConsistencyTh = 1/mnCovisibilityConsistencyTh = 3/' \
    ~/ORB_SLAM3/src/LoopClosing.cc
sed -i 's/mbLoopDetected = mnLoopNumCoincidences >= mnCovisibilityConsistencyTh/mbLoopDetected = mnLoopNumCoincidences >= 3/' \
    ~/ORB_SLAM3/src/LoopClosing.cc

# Then manually:
# 1. Restore nBoWMatches=20, nBoWInliers=15, nSim3Inliers=20, nProjMatches=50, nProjOptMatches=80
# 2. Remove spatial fallback block between [SPATIAL FALLBACK] and [END SPATIAL FALLBACK]
# 3. Remove all [LC BOW], [SPATIAL ENTRY], [SPATIAL MATCH], [SPATIAL LC] cout lines

cd ~/ORB_SLAM3 && ./build.sh
cd ~/Desktop/multi_uav_slam/ros2_ws && colcon build --packages-select iris_vslam
```

---

### **7. Why the Original Pipeline Is Sufficient**

During experimentation, the original unmodified ORB-SLAM3 pipeline with `ORBvoc.txt` was observed to fire loop closure successfully in the following cases:

| Environment | Loop size | Loop closure fires? |
|---|---|---|
| `indoor20x20` | 16×16m | ✓ |
| `indoor50x50` | 8×8 to 28×28m | ✗ |
| `indoor50x50` | 32×32m | ✓ |

This shows the original pipeline is not fundamentally broken — it fires reliably when conditions are met. The cases where it does not fire (smaller loops in `indoor50x50`) are likely due to insufficient feature overlap or geometric inconsistency between the current keyframe and candidate keyframes at the point of loop completion, rather than a vocabulary or threshold problem.

The modifications in Section 5 were an attempt to address this by lowering thresholds and adding a pose-based fallback. However, the ATE comparison (Section 2) showed this introduced more instability than it resolved. The conclusion is that for the environments used in this project, the original pipeline is both sufficient and more stable. The modifications in Section 5 may be worth revisiting in environments where loop closure consistently fails to fire despite adequate trajectory overlap.