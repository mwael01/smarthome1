Egyptian Chinese University
Faculty of Computers and Information Systems
Individual Survey Paper 3 of 12
FaceNet: A Unified Embedding for Face Recognition and Clustering (2015)
Part of: A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems

# 1. Bibliographic Reference

F. Schroff, D. Kalenichenko, and J. Philbin, “FaceNet: A Unified Embedding for Face Recognition and Clustering,” in Proc. IEEE Conf. Computer Vision and Pattern Recognition (CVPR), 2015.

# 2. Problem Addressed

Existing deep face systems often optimised intermediate bottleneck layers rather than the final embedding, and lacked an efficient way to learn compact, discriminative face representations at scale.

# 3. Method / Algorithm

Trained a deep convolutional network to directly optimise a Euclidean embedding using a triplet-loss objective with online triplet mining. Faces of the same identity are pulled together while faces of different identities are pushed apart in the embedding space. The resulting 128-byte embeddings support verification, recognition and clustering with simple distance thresholds.

# 4. Strengths

State-of-the-art accuracy (99.63% on LFW under the unrestricted protocol). Extremely compact representation. Strong influence on open-source libraries (dlib, face_recognition, etc.).

# 5. Limitations / Weaknesses

No native liveness detection. Performance depends on careful triplet selection and large training sets. Still vulnerable to presentation attacks without additional countermeasures.

# 6. Relevance to the Proposed Project

The embedding paradigm introduced by FaceNet underpins almost all modern practical face-recognition pipelines used in academic and hobbyist smart-home projects.

# 7. Key Takeaway for Future Improvement

Learning a compact metric embedding with triplet loss remains one of the most influential and practical approaches for open-source face recognition systems.
This individual survey paper is one of twelve companion documents that support the main overview paper “A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems.”
