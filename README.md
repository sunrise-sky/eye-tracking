# Eye-Tracking System (A1 Development Board)

> An embedded eye-tracking project based on grayscale sensing and infrared imaging  
> Designed for IC design / embedded competitions, with the goal of implementing eye-region detection, pupil localization, gaze direction estimation, and result visualization on the A1 development board.

---

## 1. Project Overview

This project is designed for embedded vision competition scenarios and aims to build an **infrared-based eye-tracking system**.  
The system captures eye images using a grayscale sensor, combines infrared illumination with image processing algorithms, localizes the pupil region, and determines eye movement direction based on pupil position changes.

The project focuses on the following goals:

- Implement a practical eye-tracking algorithm on a resource-constrained platform
- Improve robustness of eye-region detection under complex environments
- Build a lightweight processing pipeline suitable for deployment on the development board
- Provide a foundation for future applications such as human-computer interaction, gaze control, and assistive interaction

---

## 2. Project Objectives

### Functional Goals
- [ ] Capture grayscale eye images
- [ ] Perform infrared image preprocessing
- [ ] Detect eye / bright-spot candidate regions
- [ ] Localize the pupil
- [ ] Estimate eye movement direction (left / right / up / down / center)
- [ ] Display real-time results and debugging information
- [ ] Deploy and verify the system on the A1 development board

### Engineering Goals
- [ ] Establish a standardized multi-person collaborative repository
- [ ] Build modular code organization
- [ ] Accumulate experimental data, algorithm versions, and debugging records
- [ ] Support presentation materials, project reports, and demo videos

---

## 3. Technical Route

The current planned processing pipeline is as follows:

```text
Image Acquisition
   ↓
Infrared Illumination / Grayscale Imaging
   ↓
Image Preprocessing (filtering, enhancement, thresholding)
   ↓
Candidate Region Detection
   ↓
Pupil Region Extraction
   ↓
Pupil Center Coordinate Calculation
   ↓
Eye Movement Direction Estimation
   ↓
Result Display / Serial Output / On-screen Overlay
