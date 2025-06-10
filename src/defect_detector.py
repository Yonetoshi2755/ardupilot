#!/usr/bin/env python3
"""
Defect Detection Module for Structure Inspection
Uses computer vision and deep learning to identify structural defects
"""

import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as transforms
from PIL import Image
import json
from datetime import datetime
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
import logging
from pathlib import Path


@dataclass
class Defect:
    """Represents a detected defect"""
    type: str  # crack, spalling, corrosion, etc.
    severity: str  # minor, moderate, severe, critical
    confidence: float  # 0-1 confidence score
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    mask: Optional[np.ndarray] = None  # Segmentation mask
    area: Optional[float] = None  # Physical area in mm²
    dimensions: Optional[Dict[str, float]] = None  # width, length, depth


class DefectDetectionModel(nn.Module):
    """CNN-based defect detection model"""
    
    def __init__(self, num_classes=5):
        super(DefectDetectionModel, self).__init__()
        
        # Using a simplified architecture for demonstration
        # In production, use pre-trained models like ResNet, EfficientNet
        self.features = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            
            nn.Conv2d(128, 256, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
        )
        
        self.classifier = nn.Sequential(
            nn.Dropout(),
            nn.Linear(256 * 28 * 28, 512),
            nn.ReLU(inplace=True),
            nn.Dropout(),
            nn.Linear(512, num_classes),
        )
        
    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)
        x = self.classifier(x)
        return x


class DefectDetector:
    """Main defect detection interface"""
    
    def __init__(self, model_path: Optional[str] = None):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.logger = logging.getLogger(__name__)
        
        # Defect categories
        self.defect_classes = {
            0: "none",
            1: "crack",
            2: "spalling",
            3: "corrosion",
            4: "deformation"
        }
        
        # Severity thresholds based on defect size and type
        self.severity_thresholds = {
            "crack": {
                "minor": {"width": 0.3, "length": 50},      # mm
                "moderate": {"width": 1.0, "length": 200},
                "severe": {"width": 3.0, "length": 500},
                "critical": {"width": 5.0, "length": 1000}
            },
            "spalling": {
                "minor": {"area": 100},      # mm²
                "moderate": {"area": 1000},
                "severe": {"area": 10000},
                "critical": {"area": 50000}
            },
            "corrosion": {
                "minor": {"area": 100},
                "moderate": {"area": 1000},
                "severe": {"area": 10000},
                "critical": {"area": 50000}
            }
        }
        
        # Initialize model
        self.model = self._load_model(model_path)
        
        # Image preprocessing
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])
        ])
        
    def _load_model(self, model_path: Optional[str]) -> nn.Module:
        """Load pre-trained model or initialize new one"""
        model = DefectDetectionModel(num_classes=len(self.defect_classes))
        
        if model_path and Path(model_path).exists():
            self.logger.info(f"Loading model from {model_path}")
            model.load_state_dict(torch.load(model_path, map_location=self.device))
        else:
            self.logger.warning("No pre-trained model found, using random initialization")
        
        model.to(self.device)
        model.eval()
        return model
    
    def detect_defects(self, image_path: str, metadata: Dict) -> List[Defect]:
        """Detect defects in an image"""
        self.logger.info(f"Processing image: {image_path}")
        
        # Load image
        image = cv2.imread(image_path)
        if image is None:
            self.logger.error(f"Failed to load image: {image_path}")
            return []
        
        # Run multiple detection methods
        defects = []
        
        # 1. Deep learning based detection
        dl_defects = self._detect_with_dl(image)
        defects.extend(dl_defects)
        
        # 2. Traditional CV detection for cracks
        cv_defects = self._detect_cracks_cv(image)
        defects.extend(cv_defects)
        
        # 3. Texture analysis for surface defects
        texture_defects = self._detect_texture_anomalies(image)
        defects.extend(texture_defects)
        
        # Merge overlapping detections
        defects = self._merge_detections(defects)
        
        # Calculate physical dimensions using metadata
        defects = self._calculate_physical_dimensions(defects, image, metadata)
        
        # Assign severity based on dimensions
        defects = self._assign_severity(defects)
        
        return defects
    
    def _detect_with_dl(self, image: np.ndarray) -> List[Defect]:
        """Deep learning based defect detection"""
        defects = []
        
        # Convert to PIL Image
        pil_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        
        # Sliding window detection
        window_size = 224
        stride = 112
        
        h, w = image.shape[:2]
        
        for y in range(0, h - window_size, stride):
            for x in range(0, w - window_size, stride):
                # Extract window
                window = pil_image.crop((x, y, x + window_size, y + window_size))
                
                # Preprocess
                input_tensor = self.transform(window).unsqueeze(0).to(self.device)
                
                # Predict
                with torch.no_grad():
                    outputs = self.model(input_tensor)
                    probabilities = torch.softmax(outputs, dim=1)
                    confidence, predicted = torch.max(probabilities, 1)
                    
                # If defect detected with high confidence
                if predicted.item() != 0 and confidence.item() > 0.7:
                    defect_type = self.defect_classes[predicted.item()]
                    
                    defects.append(Defect(
                        type=defect_type,
                        severity="unknown",  # Will be determined later
                        confidence=confidence.item(),
                        bbox=(x, y, x + window_size, y + window_size)
                    ))
        
        return defects
    
    def _detect_cracks_cv(self, image: np.ndarray) -> List[Defect]:
        """Traditional computer vision crack detection"""
        defects = []
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply bilateral filter to reduce noise while keeping edges sharp
        filtered = cv2.bilateralFilter(gray, 9, 75, 75)
        
        # Edge detection
        edges = cv2.Canny(filtered, 50, 150)
        
        # Morphological operations to connect broken edges
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Filter by contour properties
            area = cv2.contourArea(contour)
            if area < 100:  # Minimum area threshold
                continue
                
            # Calculate contour properties
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h if h > 0 else 0
            
            # Cracks typically have high aspect ratio
            if aspect_ratio > 3 or aspect_ratio < 0.33:
                # Fit line to detect crack orientation
                [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # Create mask for this crack
                mask = np.zeros(gray.shape, dtype=np.uint8)
                cv2.drawContours(mask, [contour], -1, 255, -1)
                
                defects.append(Defect(
                    type="crack",
                    severity="unknown",
                    confidence=0.8,  # CV detection confidence
                    bbox=(x, y, x + w, y + h),
                    mask=mask
                ))
        
        return defects
    
    def _detect_texture_anomalies(self, image: np.ndarray) -> List[Defect]:
        """Detect surface defects using texture analysis"""
        defects = []
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Calculate Local Binary Pattern (LBP) for texture analysis
        lbp = self._calculate_lbp(gray)
        
        # Calculate texture statistics in sliding windows
        window_size = 64
        stride = 32
        
        h, w = gray.shape
        
        # Calculate global statistics
        global_mean = np.mean(lbp)
        global_std = np.std(lbp)
        
        for y in range(0, h - window_size, stride):
            for x in range(0, w - window_size, stride):
                # Extract window
                window = lbp[y:y+window_size, x:x+window_size]
                
                # Calculate local statistics
                local_mean = np.mean(window)
                local_std = np.std(window)
                
                # Detect anomalies based on statistical deviation
                mean_deviation = abs(local_mean - global_mean) / global_mean
                std_deviation = abs(local_std - global_std) / global_std
                
                # Threshold for anomaly detection
                if mean_deviation > 0.3 or std_deviation > 0.3:
                    confidence = min(mean_deviation + std_deviation, 1.0)
                    
                    defects.append(Defect(
                        type="spalling",  # Surface texture anomaly
                        severity="unknown",
                        confidence=confidence * 0.7,  # Lower confidence for texture-based
                        bbox=(x, y, x + window_size, y + window_size)
                    ))
        
        return defects
    
    def _calculate_lbp(self, image: np.ndarray, radius: int = 1, n_points: int = 8) -> np.ndarray:
        """Calculate Local Binary Pattern"""
        h, w = image.shape
        lbp = np.zeros((h-2*radius, w-2*radius), dtype=np.uint8)
        
        for i in range(radius, h-radius):
            for j in range(radius, w-radius):
                center = image[i, j]
                binary_string = ""
                
                for n in range(n_points):
                    theta = 2 * np.pi * n / n_points
                    x = int(round(i + radius * np.cos(theta)))
                    y = int(round(j - radius * np.sin(theta)))
                    
                    if image[x, y] >= center:
                        binary_string += "1"
                    else:
                        binary_string += "0"
                
                lbp[i-radius, j-radius] = int(binary_string, 2)
        
        return lbp
    
    def _merge_detections(self, defects: List[Defect]) -> List[Defect]:
        """Merge overlapping detections"""
        if not defects:
            return []
        
        # Sort by confidence
        defects.sort(key=lambda x: x.confidence, reverse=True)
        
        merged = []
        used = [False] * len(defects)
        
        for i, defect1 in enumerate(defects):
            if used[i]:
                continue
                
            # Start a new merged detection
            merged_bbox = list(defect1.bbox)
            merged_confidence = defect1.confidence
            merge_count = 1
            
            for j, defect2 in enumerate(defects[i+1:], i+1):
                if used[j] or defect1.type != defect2.type:
                    continue
                
                # Check IoU
                iou = self._calculate_iou(defect1.bbox, defect2.bbox)
                
                if iou > 0.3:  # Merge threshold
                    used[j] = True
                    # Update merged bbox
                    merged_bbox[0] = min(merged_bbox[0], defect2.bbox[0])
                    merged_bbox[1] = min(merged_bbox[1], defect2.bbox[1])
                    merged_bbox[2] = max(merged_bbox[2], defect2.bbox[2])
                    merged_bbox[3] = max(merged_bbox[3], defect2.bbox[3])
                    merged_confidence += defect2.confidence
                    merge_count += 1
            
            # Create merged defect
            merged_defect = Defect(
                type=defect1.type,
                severity="unknown",
                confidence=min(merged_confidence / merge_count * 1.2, 1.0),  # Boost confidence for multiple detections
                bbox=tuple(merged_bbox),
                mask=defect1.mask  # Keep first mask
            )
            
            merged.append(merged_defect)
            used[i] = True
        
        return merged
    
    def _calculate_iou(self, bbox1: Tuple, bbox2: Tuple) -> float:
        """Calculate Intersection over Union"""
        x1 = max(bbox1[0], bbox2[0])
        y1 = max(bbox1[1], bbox2[1])
        x2 = min(bbox1[2], bbox2[2])
        y2 = min(bbox1[3], bbox2[3])
        
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        
        area1 = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
        area2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
        
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0
    
    def _calculate_physical_dimensions(self, defects: List[Defect], image: np.ndarray, 
                                     metadata: Dict) -> List[Defect]:
        """Convert pixel measurements to physical dimensions"""
        # Extract camera parameters from metadata
        # This is simplified - real implementation would use proper camera calibration
        altitude = metadata.get("altitude", 10)  # meters
        focal_length = metadata.get("focal_length", 24)  # mm
        sensor_width = metadata.get("sensor_width", 36)  # mm
        
        h, w = image.shape[:2]
        
        # Calculate ground sample distance (GSD) - mm per pixel
        gsd = (altitude * 1000 * sensor_width) / (focal_length * w)
        
        for defect in defects:
            bbox = defect.bbox
            
            # Calculate dimensions in mm
            width_px = bbox[2] - bbox[0]
            height_px = bbox[3] - bbox[1]
            
            width_mm = width_px * gsd
            height_mm = height_px * gsd
            
            defect.dimensions = {
                "width": width_mm,
                "length": max(width_mm, height_mm),
                "area": width_mm * height_mm
            }
            
            defect.area = width_mm * height_mm
        
        return defects
    
    def _assign_severity(self, defects: List[Defect]) -> List[Defect]:
        """Assign severity levels based on defect dimensions"""
        for defect in defects:
            if defect.type not in self.severity_thresholds:
                defect.severity = "unknown"
                continue
            
            thresholds = self.severity_thresholds[defect.type]
            dimensions = defect.dimensions
            
            if not dimensions:
                defect.severity = "unknown"
                continue
            
            # Determine severity
            severity = "minor"
            
            for level in ["critical", "severe", "moderate", "minor"]:
                if level not in thresholds:
                    continue
                    
                threshold = thresholds[level]
                
                # Check each dimension against threshold
                exceeds_threshold = False
                
                if defect.type == "crack":
                    if (dimensions.get("width", 0) >= threshold.get("width", float('inf')) or
                        dimensions.get("length", 0) >= threshold.get("length", float('inf'))):
                        exceeds_threshold = True
                        
                elif defect.type in ["spalling", "corrosion"]:
                    if dimensions.get("area", 0) >= threshold.get("area", float('inf')):
                        exceeds_threshold = True
                
                if exceeds_threshold:
                    severity = level
                    break
            
            defect.severity = severity
        
        return defects
    
    def visualize_detections(self, image_path: str, defects: List[Defect], 
                           output_path: str) -> None:
        """Visualize detected defects on image"""
        image = cv2.imread(image_path)
        
        # Color mapping for severity
        severity_colors = {
            "minor": (0, 255, 0),      # Green
            "moderate": (0, 255, 255),  # Yellow
            "severe": (0, 165, 255),    # Orange
            "critical": (0, 0, 255),    # Red
            "unknown": (128, 128, 128)  # Gray
        }
        
        # Draw defects
        for defect in defects:
            x1, y1, x2, y2 = defect.bbox
            color = severity_colors.get(defect.severity, (128, 128, 128))
            
            # Draw bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            
            # Add label
            label = f"{defect.type} ({defect.severity}) {defect.confidence:.2f}"
            cv2.putText(image, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw dimensions if available
            if defect.dimensions:
                dims_text = f"W:{defect.dimensions['width']:.1f}mm"
                cv2.putText(image, dims_text, (x1, y2 + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Save annotated image
        cv2.imwrite(output_path, image)
        self.logger.info(f"Saved annotated image to {output_path}")


def main():
    """Test the defect detector"""
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python defect_detector.py <image_path>")
        sys.exit(1)
    
    # Configure logging
    logging.basicConfig(level=logging.INFO)
    
    # Initialize detector
    detector = DefectDetector()
    
    # Test metadata
    metadata = {
        "altitude": 10,
        "focal_length": 24,
        "sensor_width": 36
    }
    
    # Detect defects
    image_path = sys.argv[1]
    defects = detector.detect_defects(image_path, metadata)
    
    # Print results
    print(f"\nDetected {len(defects)} defects:")
    for i, defect in enumerate(defects):
        print(f"\nDefect {i+1}:")
        print(f"  Type: {defect.type}")
        print(f"  Severity: {defect.severity}")
        print(f"  Confidence: {defect.confidence:.2f}")
        if defect.dimensions:
            print(f"  Dimensions: {defect.dimensions}")
    
    # Visualize
    output_path = image_path.replace('.jpg', '_annotated.jpg')
    detector.visualize_detections(image_path, defects, output_path)


if __name__ == "__main__":
    main()