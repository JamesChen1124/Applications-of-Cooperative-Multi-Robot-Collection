# Applications of Cooperative Multi-Robot Collection

## Overview
This project designs an automated multi-robot system for ball collection (tennis, badminton, table tennis)[tennis focused] using YOLOv8n, Python, and Arduino.  
Robots are equipped with ESP32-CAM for vision input and controlled via PID-based servo mechanisms.  

## Project Structure
- `src/` – Python and Arduino source code  
- `cad/` – Circuit diagrams and CAD files  
- `docs/` – Reports and project documents [Mandarin version]  
- `dataset/` – Small sample of training images (full dataset available via Roboflow)  

## Dataset
A sample dataset is provided in `/dataset/`.  
The full dataset (9,500 images) is hosted on Roboflow:  
👉 [View and Download Dataset][https://universe.roboflow.com/your-dataset-link](https://app.roboflow.com/jameschen/tennis-table-tennis-badminton/7))

## Features
- Multi-robot collaboration for object retrieval
- YOLOv8n vision model for real-time detection
- Vision-based servo control using PID

## Hardware
- Arduino Mega / ESP32-CAM
- Mobile robot with robotic arm
- Bluetooth module

