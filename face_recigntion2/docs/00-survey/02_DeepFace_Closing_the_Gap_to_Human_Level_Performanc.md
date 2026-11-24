Egyptian Chinese University
Faculty of Computers and Information Systems
Individual Survey Paper 2 of 12
DeepFace: Closing the Gap to Human-Level Performance (2014)
Part of: A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems

# 1. Bibliographic Reference

Y. Taigman, M. Yang, M. Ranzato, and L. Wolf, “DeepFace: Closing the Gap to Human-Level Performance in Face Verification,” in Proc. IEEE Conf. Computer Vision and Pattern Recognition (CVPR), 2014.

# 2. Problem Addressed

Unconstrained face verification accuracy lagged significantly behind human performance on datasets such as Labeled Faces in the Wild (LFW).

# 3. Method / Algorithm

Employed explicit 3D face modelling for piecewise affine alignment, followed by a nine-layer deep neural network (more than 120 million parameters) trained on a large identity-labelled dataset of approximately four million images. The resulting representation was used with a simple classifier.

# 4. Strengths

Achieved 97.35% accuracy on LFW, approaching human-level performance and reducing the error of prior state-of-the-art by more than 27%. Demonstrated the power of deep representation learning combined with geometric alignment.

# 5. Limitations / Weaknesses

Very large model size and training data requirements. Limited public deployment code at the time. No built-in liveness or anti-spoofing protection.

# 6. Relevance to the Proposed Project

Established that deep CNNs could reach near-human verification accuracy, motivating the embedding-based pipelines used in contemporary open-source face-recognition libraries.

# 7. Key Takeaway for Future Improvement

Accurate alignment plus large-scale supervised deep learning is a powerful combination; subsequent work focused on making similar accuracy achievable with far smaller models.
This individual survey paper is one of twelve companion documents that support the main overview paper “A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems.”
