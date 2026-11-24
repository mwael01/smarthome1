Egyptian Chinese University
Faculty of Computers and Information Systems
Individual Survey Paper 5 of 12
MobileFaceNets: Efficient CNNs for Mobile Devices (2018)
Part of: A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems

# 1. Bibliographic Reference

S. Chen, Y. Liu, X. Gao, and Z. Han, “MobileFaceNets: Efficient CNNs for Accurate Real-Time Face Verification on Mobile Devices,” in Biometric Recognition (CCBR), 2018.

# 2. Problem Addressed

High-accuracy face-verification models were too large and slow for real-time deployment on mobile and embedded devices.

# 3. Method / Algorithm

Designed a family of extremely efficient CNNs (MobileFaceNets) with fewer than 1 million parameters, based on inverted residual bottlenecks with carefully chosen expansion factors and a global depth-wise convolution. Trained with ArcFace loss on refined large-scale data.

# 4. Strengths

Achieved 99.55% accuracy on LFW with a 4 MB model. Real-time inference on mobile phones (as low as 18 ms). Significantly better accuracy–speed trade-off than generic MobileNet variants for the face-verification task.

# 5. Limitations / Weaknesses

Slight accuracy gap compared with the largest research models. Still benefits from a separate anti-spoofing stage for security-critical deployments.

# 6. Relevance to the Proposed Project

Demonstrates that high verification accuracy is attainable on resource-constrained platforms, making face recognition practical for edge devices or lightweight simulation backends.

# 7. Key Takeaway for Future Improvement

Task-specific lightweight architectures can deliver near-SOTA accuracy at a fraction of the computational cost of general-purpose networks.
This individual survey paper is one of twelve companion documents that support the main overview paper “A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems.”
