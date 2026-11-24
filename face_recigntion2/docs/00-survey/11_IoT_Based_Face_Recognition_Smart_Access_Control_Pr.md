Egyptian Chinese University
Faculty of Computers and Information Systems
Individual Survey Paper 11 of 12
IoT-Based Face Recognition Smart Access Control Prototypes
Part of: A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems

# 1. Bibliographic Reference

Representative student and academic prototypes using ESP32-CAM or Raspberry Pi with face recognition and solenoid/relay locks (2023–2025).

# 2. Problem Addressed

Commercial smart locks are expensive and closed; low-cost open alternatives that combine face recognition with physical actuation are needed for research and education.

# 3. Method / Algorithm

Typical architecture: camera module (ESP32-CAM or USB webcam on Raspberry Pi), face detection/recognition via OpenCV + dlib or lightweight CNNs, decision logic that drives a relay controlling a solenoid door lock, optional keypad or RFID as secondary factor, and basic logging or Telegram/email notifications.

# 4. Strengths

Low hardware cost, fully reproducible, demonstrates end-to-end sensing–decision–actuation pipeline. Many implementations achieve acceptable accuracy under controlled lighting.

# 5. Limitations / Weaknesses

Most omit robust liveness detection, encrypted template storage and tamper-evident logging. Performance degrades under uncontrolled illumination or large pose variation. Security of the communication channel is often weak.

# 6. Relevance to the Proposed Project

These prototypes prove that a complete software-plus-actuation system is feasible at student-project scale and supply concrete architectural patterns the proposed system can refine.

# 7. Key Takeaway for Future Improvement

The missing pieces in most existing prototypes—liveness, secure storage and structured logging—are exactly the gaps the present project aims to close.
This individual survey paper is one of twelve companion documents that support the main overview paper “A Literature Survey on Multi-Sensor Face-Recognition-Based Smart Home Security Systems.”
