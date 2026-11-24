Egyptian Chinese University
Faculty of Computers and Information Systems
Individual Survey Paper 9 of 12
Enhancing Smart Home Security with DCNNs and Multiple Cameras (2024)
Part of: A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems

# 1. Bibliographic Reference

Recent work on multi-camera deep convolutional networks for smart-home occupant versus intruder classification (Wireless Personal Communications, 2024).

# 2. Problem Addressed

Single-camera systems suffer from limited field of view and are easily defeated by occlusion or adverse angles; continuous recording also raises privacy and storage concerns.

# 3. Method / Algorithm

Deploys three cameras capturing different viewpoints. Images are classified by deep convolutional networks (VGG16, VGG19, DenseNet) to distinguish known occupants from intruders. Remote disable capability is provided for authorised users.

# 4. Strengths

Reported very high classification accuracy (approximately 99.8% in the authors’ experiments). Multi-view fusion reduces blind spots. Demonstrates that deep models can be applied to practical residential multi-camera setups.

# 5. Limitations / Weaknesses

Relatively heavy backbone networks. Limited discussion of liveness detection and of the computational cost of running multiple high-capacity models simultaneously.

# 6. Relevance to the Proposed Project

Directly supports the dual- (or multi-) camera design choice of the proposed project and shows that multi-view analysis improves reliability.

# 7. Key Takeaway for Future Improvement

Multiple viewpoints combined with modern CNNs can yield high accuracy for occupant/intruder discrimination; lightweight backbones should be preferred for resource-constrained deployments.
This individual survey paper is one of twelve companion documents that support the main overview paper “A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems.”
