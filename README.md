# Machine Vision Sorting System

Real-time object detection and classification system for an industrial sorting line using a Basler industrial camera, OpenCV, and C++.

---

## Project Overview

This project implements a real-time machine vision system designed to detect and classify objects moving on a conveyor belt based on their surface area.

The system:

- Captures images using a **Basler industrial camera**
- Processes frames using **OpenCV**
- Detects object contours
- Calculates object area
- Classifies objects as **Small** or **Big**
- Activates the appropriate **sorting pusher mechanism**
- Operates in synchronization with an industrial sorting line controller

The entire system runs in real time and communicates with the sorting line controller via serial communication.

---

## Example Output

Below is an example of object classification and visualization:

![Classification Example](classification-example.PNG)

The system:
- Detects object contours
- Computes area
- Labels each object directly on the image
- Selects the appropriate pusher based on classification

---

## Hardware Architecture

The system consists of:

- **Basler Industrial Camera** – Image acquisition  
- **Industrial Sorting Line Controller** – Conveyor control and synchronization  
- **Photocell Sensors** – Object detection  
- **Pushers (3 outputs)** – Physical object sorting  
- **PC (C++ application)** – Image processing and decision logic  

The controller:

- Tracks object beginning and end positions  
- Controls conveyor belt position  
- Synchronizes camera triggering  
- Executes pusher activation at precise positions  

---

## Image Processing Pipeline

For each acquired frame:

1. Convert image to grayscale  
2. Apply Gaussian blur for noise reduction  
3. Apply conveyor belt region mask (ROI)  
4. Perform Otsu thresholding  
5. Apply morphological closing  
6. Detect object contours  
7. Compute contour area  
8. Classify object:
   - `Small` (area < threshold)
   - `Big` (area > threshold)
9. Overlay classification result on image  

---

## Sorting Logic

After classification:

| Classification | Pusher |
|---------------|--------|
| Small         | Pusher 2 |
| Big           | Pusher 3 |
| Unknown       | Pusher 1 |

Object position tracking ensures precise timing between detection and physical sorting.

---

## Technologies Used

- **C++**
- **OpenCV**
- **Basler Pylon SDK**
- **Serial Communication (Sorting Line Controller)**

---

## 📄 Project Report

A detailed description of the system architecture, algorithm design, and hardware setup is available in:

