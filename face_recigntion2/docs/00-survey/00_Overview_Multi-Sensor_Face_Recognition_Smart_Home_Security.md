Egyptian Chinese University
Faculty of Computers and Information Systems
A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems
Graduation Project — Survey Paper (Overview)
Prepared by:
Khaled Niazy Yassein — Cyber Security
Khaled Naser Mansour — Cyber Security
Abdelraouf Diaa Elnagar — Cyber Security
Yousef Amer Sadawy — Cyber Security
Ahmed Mohamed Hikal — Artificial Intelligence
Pola Lazem Shafek — Data Science
Academic Supervisor: Dr. Ayman Mohamed

# Abstract

Traditional home access control relies on physical or knowledge-based credentials (keys, PIN codes, RFID cards). These mechanisms authenticate an object or a code rather than the identity of the person presenting it, leaving them vulnerable to loss, theft, duplication, and unauthorized sharing. Recent advances in deep learning have made face recognition a practical, contactless alternative for identity-based access control. At the same time, modern smart-home security requires more than identity verification alone: environmental threats (smoke, flame) and physical intrusion (motion) must also be monitored. This survey reviews the evolution of access-control technology—from PIN/RFID and fingerprint systems to deep-learning-based face recognition—and examines the core techniques underlying modern face recognition (Viola–Jones, DeepFace, FaceNet, ArcFace, MobileFaceNets, RetinaFace), liveness/anti-spoofing methods, multi-camera setups, and environmental sensors (smoke and flame detection). Twelve representative works are analysed in detail in companion papers. A comparative summary is provided, followed by a gap analysis that motivates the proposed multi-sensor Smart Home Security System with Face Recognition, designed for software implementation with high-quality simulation and render support.

# 1. Introduction

The convergence of the Internet of Things (IoT) and artificial intelligence has reshaped home security from simple mechanical protection into automated, data-driven systems. Traditional entry methods—metal keys, PIN keypads, and RFID cards—remain common because they are inexpensive and easy to deploy. However, all of them share a fundamental weakness: they verify possession of a credential, not the identity of the person holding it. Anyone in possession of a valid key, card, or code can gain entry, regardless of authorization.
Biometric authentication addresses this weakness by binding access to a trait intrinsic to the individual. Among biometric modalities, face recognition is particularly attractive for home security because it is contactless, works at a distance, and requires no active cooperation beyond looking toward a camera. At the same time, a complete residential security solution must also detect environmental hazards (smoke, flame) and physical intrusion (motion), and must actuate a gate or door only after successful identity verification and liveness checks.
This overview paper positions the proposed graduation project within the broader landscape of access-control and smart-home security research. It summarises prior systems and techniques, identifies recurring limitations, and frames the specific gaps that the project addresses. Detailed analyses of twelve key papers appear in the companion individual survey papers (Papers 1–12).

# 2. Related Work

## 2.1 Credential-Based Access Control (PIN and RFID)

Early electronic access-control systems relied on PIN keypads and RFID or magnetic key cards. These systems are inexpensive and simple to install. Their central weakness is that they authenticate a credential rather than a person: a code can be shared or guessed, and a card can be cloned or lent—a failure mode often referred to as “credential lending.”

## 2.2 Fingerprint Biometric Systems

Fingerprint-based locks improved on PIN/RFID systems by binding access to a physical biometric trait. However, they require direct physical contact with a sensor, raise hygiene concerns, can be degraded by dirt, moisture or skin conditions, and offer no way to verify a visitor’s identity before the person reaches the sensor. These constraints motivated interest in contactless modalities such as face recognition.

## 2.3 Foundations of Face Detection and Recognition

Face detection and recognition research progressed from classical, handcrafted-feature methods to deep learning. The Viola–Jones detector (2001) introduced a fast cascade classifier based on Haar-like features, enabling real-time face detection on limited hardware; it only locates a face and performs no identity verification. The shift to deep learning produced embedding-based recognition: DeepFace (2014) used a deep convolutional network with 3D face alignment to approach human-level verification accuracy; FaceNet (2015) introduced a triplet-loss objective that maps each face to a compact embedding vector; ArcFace (2019) further improved discriminability via an additive angular margin loss; MobileFaceNets (2018) demonstrated that high accuracy is possible with models under 1 million parameters suitable for mobile and embedded devices; and RetinaFace (2020) provided a practical single-shot multi-level face detector with landmark prediction. These works form the algorithmic backbone of most modern open-source face-recognition pipelines.

## 2.4 Liveness Detection and Anti-Spoofing

A recurring shortcoming across academic face-recognition prototypes is the omission of liveness (anti-spoofing) protection, leaving systems vulnerable to printed photographs or replayed videos. Modern approaches use texture analysis, depth-map estimation, remote photoplethysmography (rPPG), and multi-modal fusion. Recent surveys emphasise domain generalisation and open-set evaluation as critical open problems.

## 2.5 Multi-Camera and Motion Detection

Multi-camera systems improve coverage and reduce blind spots. Recent work combines deep convolutional networks with multiple viewpoints to detect occupants versus intruders and to support cross-camera tracking. Motion detection remains a lightweight first-stage filter that triggers higher-cost face recognition only when activity is present.

## 2.6 Environmental Sensors (Smoke and Flame)

Smoke and flame detection have evolved from simple threshold sensors to vision-based and multi-sensor fusion approaches. Modern systems employ lightweight CNNs or YOLO-style detectors for early fire detection from camera streams, often fused with gas, temperature or particulate sensors. Integrating these signals with access-control logic allows a smart-home system to both authenticate residents and protect them from environmental hazards.

## 2.7 Commercial and Integrated Prototypes

Commercial smart locks and doorbells offer polished user experiences and cloud notifications but are typically closed, proprietary, costly, and difficult to customise. Academic and student prototypes based on ESP32-CAM, Raspberry Pi, OpenCV and dlib demonstrate that low-cost, reproducible systems are feasible, yet many still omit robust liveness checks, encrypted template storage, and structured tamper-evident logging.

# 3. Comparative Summary of Twelve Representative Works

Table 1 summarises the twelve works reviewed in the companion individual survey papers along the dimensions most relevant to a multi-sensor smart-home access-control system.
Table 1. Comparison of the twelve reviewed works.

| # | Work | Core Technique | Strengths | Key Limitations |
|---|---|---|---|---|
| 1 | Viola–Jones (2001) | Haar cascade | Fast, real-time, low compute | Detection only; no identity; pose/lighting sensitive |
| 2 | DeepFace (2014) | Deep CNN + 3D align | Near human-level verification | Large model; limited public code; no liveness |
| 3 | FaceNet (2015) | Triplet-loss embedding | Compact 128-D embeddings; strong open-source impact | No native liveness; vulnerable to photo/video spoof |
| 4 | ArcFace (2019) | Angular margin loss | Highly discriminative features; SOTA accuracy | Requires careful training; still needs anti-spoofing |
| 5 | MobileFaceNets (2018) | Lightweight CNN | <1M params; real-time on mobile/edge | Slight accuracy trade-off vs large models |
| 6 | RetinaFace (2020) | Single-shot detector + landmarks | Accurate detection under pose/occlusion | Detection stage only; recognition separate |
| 7 | rPPG / Depth FAS methods | Physiological + depth cues | Effective against print/replay attacks | Sensitive to motion/lighting; extra compute |
| 8 | DL FAS Surveys (2023) | Survey of CNN/Transformer FAS | Comprehensive taxonomy & benchmarks | Domain gap remains an open challenge |
| 9 | Multi-Camera DCNN (2024) | Multi-view DCNN classification | High accuracy occupant vs intruder | Heavy models; limited liveness discussion |
| 10 | YOLO-HF / Fire Vision | YOLO-style smoke/flame detection | Early detection; lightweight variants | False positives under complex lighting |
| 11 | IoT Face Access Prototypes | ESP32 / Pi + face + lock | Low-cost, reproducible, real actuation | Often lack liveness & secure logging |
| 12 | Two-Layer Multi-Cam AI | AI intrusion + cross-cam tracking | Reduces continuous recording load | Complexity of multi-camera coordination |

Detailed summaries of each of the twelve works appear in the companion individual survey papers.

# 4. Gap Analysis

Across the reviewed literature the following gaps recur:
- Credential-based systems (PIN/RFID) authenticate an object, not a person, and remain exposed to loss, theft and duplication.
- Fingerprint systems require physical contact and cannot verify identity from a distance.
- Commercial face-recognition locks are closed, costly and non-customisable.
- Many academic face-recognition prototypes omit robust liveness/anti-spoofing protection.
- Few low-cost open prototypes combine face recognition with motion detection, smoke/flame sensing, encrypted biometric storage and tamper-evident access logging in a single coherent architecture.
- Multi-camera coordination and software-in-the-loop simulation of a residential entrance are rarely treated as first-class design concerns in student-scale projects.

# 5. Proposed System Overview

The proposed Smart Home Security System is designed as a lightweight, open and extensible architecture suitable for a graduation project. It focuses on a residential main-entrance scenario and incorporates the following core capabilities:
- Face detection and recognition using a modern embedding pipeline (inspired by FaceNet / ArcFace / MobileFaceNets) with open-source libraries.
- Basic liveness / anti-spoofing checks to reduce photo and video replay attacks.
- Motion detection as a lightweight trigger for higher-cost recognition.
- Smoke and flame detection (vision-based and/or simple sensor thresholds) for environmental safety.
- Dual-camera concept (entrance + secondary view) for improved coverage.
- Gate / door actuation logic (simulated or relay-controlled).
- Secure logging of access events and environmental alerts with basic dashboard / notification support.
The system is intended to be implemented primarily in software, with a high-quality simulation and 3D render of a residential entrance used for demonstration and evaluation. The architecture remains compatible with future mapping onto edge devices (Raspberry Pi / ESP32-class hardware) should physical deployment be required.

# 6. Conclusion

This survey traced the evolution of home access control from credential-based systems through contact-based biometrics to deep-learning face recognition, and examined complementary sensing modalities (motion, smoke, flame) and multi-camera techniques. Across commercial and academic solutions two gaps recur: closedness and cost in commercial products, and insufficient attention to anti-spoofing, multi-sensor fusion and secure logging in many open prototypes. The proposed multi-sensor Smart Home Security System with Face Recognition is designed to address these gaps within the realistic scope of a graduation project, combining a practical face-recognition pipeline with liveness checks, environmental sensing, dual-camera coverage and tamper-evident logging inside an open, simulation-friendly architecture.

# References

[1] P. Viola and M. Jones, “Rapid Object Detection using a Boosted Cascade of Simple Features,” in Proc. IEEE Conf. Computer Vision and Pattern Recognition (CVPR), 2001.
[2] Y. Taigman, M. Yang, M. Ranzato, and L. Wolf, “DeepFace: Closing the Gap to Human-Level Performance in Face Verification,” in Proc. IEEE Conf. Computer Vision and Pattern Recognition (CVPR), 2014.
[3] F. Schroff, D. Kalenichenko, and J. Philbin, “FaceNet: A Unified Embedding for Face Recognition and Clustering,” in Proc. IEEE Conf. Computer Vision and Pattern Recognition (CVPR), 2015.
[4] J. Deng, J. Guo, N. Xue, and S. Zafeiriou, “ArcFace: Additive Angular Margin Loss for Deep Face Recognition,” in Proc. IEEE/CVF Conf. Computer Vision and Pattern Recognition (CVPR), 2019.
[5] S. Chen, Y. Liu, X. Gao, and Z. Han, “MobileFaceNets: Efficient CNNs for Accurate Real-Time Face Verification on Mobile Devices,” in Biometric Recognition (CCBR), 2018.
[6] J. Deng, J. Guo, E. Ververas, I. Kotsia, and S. Zafeiriou, “RetinaFace: Single-Shot Multi-Level Face Localisation in the Wild,” in Proc. IEEE/CVF Conf. Computer Vision and Pattern Recognition (CVPR), 2020.
[7] Representative works on rPPG- and depth-based face anti-spoofing (see companion Paper 7).
[8] Z. Yu et al., “Deep Learning for Face Anti-Spoofing: A Survey,” IEEE Trans. Pattern Analysis and Machine Intelligence, 2023.
[9] Works on multi-camera DCNN smart-home security (see companion Paper 9), 2024.
[10] YOLO-HF and related vision-based early home-fire detection methods (see companion Paper 10).
[11] IoT-based face-recognition access-control prototypes using ESP32 / Raspberry Pi (see companion Paper 11).
[12] Two-layer AI-integrated multi-camera intrusion detection systems (see companion Paper 12), 2024.
Note: Full bibliographic details and in-depth analysis of each of the twelve works are provided in the individual companion survey papers (Papers 1–12). Additional references to OpenCV, dlib, TensorFlow and related libraries appear in those papers.
