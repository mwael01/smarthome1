Egyptian Chinese University
Faculty of Computers and Information Systems
Individual Survey Paper 7 of 12
rPPG- and Depth-Based Face Anti-Spoofing Methods
Part of: A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems

# 1. Bibliographic Reference

Representative works: Liu et al., “Learning Deep Models for Face Anti-Spoofing: Binary or Auxiliary Supervision,” CVPR 2018; and subsequent rPPG / depth-map approaches summarised in recent FAS surveys.

# 2. Problem Addressed

Face-recognition systems can be deceived by printed photographs, video replays or 3D masks (presentation attacks).

# 3. Method / Algorithm

Two complementary families of cues are widely used: (1) estimated depth maps that expose the planar nature of print/replay attacks, and (2) remote photoplethysmography (rPPG) signals that recover subtle skin-colour changes caused by blood flow, which are absent in spoofs. These cues are often combined with texture CNNs under auxiliary supervision.

# 4. Strengths

Depth and rPPG provide strong physiological and geometric evidence of liveness. Many methods achieve low error rates on standard intra-dataset protocols for print and replay attacks.

# 5. Limitations / Weaknesses

rPPG is sensitive to motion and illumination. Depth estimation from a single RGB image is an ill-posed problem and can itself be attacked. Cross-dataset generalisation remains challenging.

# 6. Relevance to the Proposed Project

Liveness detection is a critical missing component in many academic smart-home prototypes; incorporating even a lightweight anti-spoofing stage substantially raises the security bar.

# 7. Key Takeaway for Future Improvement

A practical system should combine at least one liveness cue (depth, rPPG or challenge-response) with the recognition pipeline rather than treating anti-spoofing as an optional afterthought.
This individual survey paper is one of twelve companion documents that support the main overview paper “A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems.”
