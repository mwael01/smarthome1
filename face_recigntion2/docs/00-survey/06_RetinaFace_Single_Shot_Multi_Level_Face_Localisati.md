Egyptian Chinese University
Faculty of Computers and Information Systems
Individual Survey Paper 6 of 12
RetinaFace: Single-Shot Multi-Level Face Localisation (2020)
Part of: A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems

# 1. Bibliographic Reference

J. Deng, J. Guo, E. Ververas, I. Kotsia, and S. Zafeiriou, “RetinaFace: Single-Shot Multi-Level Face Localisation in the Wild,” in Proc. IEEE/CVF Conf. Computer Vision and Pattern Recognition (CVPR), 2020.

# 2. Problem Addressed

Existing face detectors struggled with faces of varying scales, poses and occlusion levels in unconstrained (“in-the-wild”) images.

# 3. Method / Algorithm

Proposed a single-shot, multi-level face localisation method that jointly predicts face bounding boxes, five facial landmarks and a 3D dense correspondence. Built on a feature-pyramid style architecture and trained with multi-task losses.

# 4. Strengths

State-of-the-art detection performance on WIDER FACE, especially under hard conditions (small faces, occlusion, extreme pose). Provides accurate landmarks useful for subsequent alignment. Open-source implementations are widely used.

# 5. Limitations / Weaknesses

Focuses on detection and localisation; recognition is performed by a separate model. Computational cost is higher than the original Viola–Jones cascade.

# 6. Relevance to the Proposed Project

RetinaFace (and its lightweight MobileNet variants) is a practical modern replacement for classical detectors in contemporary face-recognition pipelines.

# 7. Key Takeaway for Future Improvement

Joint multi-task prediction of boxes, landmarks and dense geometry yields robust localisation that benefits the entire recognition pipeline.
This individual survey paper is one of twelve companion documents that support the main overview paper “A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems.”
