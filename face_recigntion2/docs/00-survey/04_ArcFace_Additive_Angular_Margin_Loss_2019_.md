Egyptian Chinese University
Faculty of Computers and Information Systems
Individual Survey Paper 4 of 12
ArcFace: Additive Angular Margin Loss (2019)
Part of: A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems

# 1. Bibliographic Reference

J. Deng, J. Guo, N. Xue, and S. Zafeiriou, “ArcFace: Additive Angular Margin Loss for Deep Face Recognition,” in Proc. IEEE/CVF Conf. Computer Vision and Pattern Recognition (CVPR), 2019.

# 2. Problem Addressed

Softmax-based losses do not explicitly enforce large angular margins between classes, limiting the discriminative power of learned face features.

# 3. Method / Algorithm

Proposed an Additive Angular Margin Loss (ArcFace) that has a clear geometric interpretation corresponding to the geodesic distance on a hypersphere. The margin is added directly in the angular space, encouraging features of the same identity to cluster tightly while different identities are separated by a larger angular gap.

# 4. Strengths

Consistently strong performance across multiple large-scale benchmarks. Easy to implement with negligible computational overhead. Widely adopted in open-source frameworks (InsightFace).

# 5. Limitations / Weaknesses

Still requires large clean training data. Sensitive to label noise (later addressed by sub-centre ArcFace variants). Does not itself solve presentation attacks.

# 6. Relevance to the Proposed Project

ArcFace (and its derivatives) is currently one of the most accurate and widely used loss functions in production-grade and open-source face-recognition systems.

# 7. Key Takeaway for Future Improvement

Angular-margin losses provide a principled and effective way to obtain highly discriminative face embeddings suitable for high-security access-control applications.
This individual survey paper is one of twelve companion documents that support the main overview paper “A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems.”
