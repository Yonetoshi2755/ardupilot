#!/usr/bin/env python3
"""
Report Generator for Structure Inspection
Creates comprehensive inspection reports with defect analysis and visualizations
"""

import json
import os
import math
from datetime import datetime
from typing import List, Dict, Any, Optional
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from dataclasses import dataclass, asdict
import pandas as pd
from jinja2 import Template
import folium
from folium import plugins
import base64
from io import BytesIO
from PIL import Image


@dataclass
class InspectionSummary:
    """Summary statistics for an inspection"""
    total_waypoints: int
    waypoints_completed: int
    total_defects: int
    critical_defects: int
    severe_defects: int
    moderate_defects: int
    minor_defects: int
    inspection_duration: float  # minutes
    total_distance: float  # meters
    average_battery_used: float  # percentage
    weather_conditions: Dict[str, Any]


class ReportGenerator:
    """Generates comprehensive inspection reports"""
    
    def __init__(self, output_dir: str = "data/reports"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # Report templates
        self.html_template = """
<!DOCTYPE html>
<html>
<head>
    <title>Structure Inspection Report - {{ report_date }}</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .header { background-color: #2c3e50; color: white; padding: 20px; }
        .summary { background-color: #ecf0f1; padding: 15px; margin: 20px 0; }
        .defect { border: 1px solid #bdc3c7; margin: 10px 0; padding: 10px; }
        .critical { border-color: #e74c3c; background-color: #ffe6e6; }
        .severe { border-color: #e67e22; background-color: #fff2e6; }
        .moderate { border-color: #f39c12; background-color: #fffbe6; }
        .minor { border-color: #27ae60; background-color: #e6ffe6; }
        .image-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 10px; }
        .stats-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 15px; }
        .stat-card { background: white; padding: 15px; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        .map { width: 100%; height: 500px; margin: 20px 0; }
        table { width: 100%; border-collapse: collapse; }
        th, td { padding: 8px; text-align: left; border-bottom: 1px solid #ddd; }
        th { background-color: #34495e; color: white; }
    </style>
</head>
<body>
    <div class="header">
        <h1>Structure Inspection Report</h1>
        <p>Generated: {{ report_date }}</p>
        <p>Structure: {{ structure_name }}</p>
    </div>
    
    <div class="summary">
        <h2>Executive Summary</h2>
        <div class="stats-grid">
            <div class="stat-card">
                <h3>Total Defects</h3>
                <p style="font-size: 24px; color: #e74c3c;">{{ summary.total_defects }}</p>
            </div>
            <div class="stat-card">
                <h3>Critical Issues</h3>
                <p style="font-size: 24px; color: #c0392b;">{{ summary.critical_defects }}</p>
            </div>
            <div class="stat-card">
                <h3>Inspection Duration</h3>
                <p style="font-size: 24px;">{{ "%.1f"|format(summary.inspection_duration) }} min</p>
            </div>
            <div class="stat-card">
                <h3>Coverage</h3>
                <p style="font-size: 24px;">{{ "%.1f"|format(summary.waypoints_completed / summary.total_waypoints * 100) }}%</p>
            </div>
        </div>
    </div>
    
    <h2>Defect Distribution</h2>
    <img src="data:image/png;base64,{{ defect_chart }}" alt="Defect Distribution" style="max-width: 600px;">
    
    <h2>Inspection Path</h2>
    <div id="map" class="map">{{ map_html|safe }}</div>
    
    <h2>Detected Defects</h2>
    <table>
        <tr>
            <th>ID</th>
            <th>Type</th>
            <th>Severity</th>
            <th>Location</th>
            <th>Confidence</th>
            <th>Dimensions</th>
            <th>Notes</th>
        </tr>
        {% for defect in defects %}
        <tr class="{{ defect.severity }}">
            <td>{{ loop.index }}</td>
            <td>{{ defect.type }}</td>
            <td>{{ defect.severity|upper }}</td>
            <td>({{ "%.6f"|format(defect.location[0]) }}, {{ "%.6f"|format(defect.location[1]) }})</td>
            <td>{{ "%.2f"|format(defect.confidence * 100) }}%</td>
            <td>
                {% if defect.dimensions %}
                W: {{ "%.1f"|format(defect.dimensions.width) }}mm, 
                L: {{ "%.1f"|format(defect.dimensions.length) }}mm
                {% else %}
                N/A
                {% endif %}
            </td>
            <td>{{ defect.notes }}</td>
        </tr>
        {% endfor %}
    </table>
    
    <h2>Defect Images</h2>
    <div class="image-grid">
        {% for defect in defects %}
        {% if defect.image_data %}
        <div class="defect {{ defect.severity }}">
            <img src="data:image/jpeg;base64,{{ defect.image_data }}" style="width: 100%;">
            <p><strong>{{ defect.type|title }}</strong> - {{ defect.severity|upper }}</p>
            <p>Location: {{ loop.index }}</p>
        </div>
        {% endif %}
        {% endfor %}
    </div>
    
    <h2>Recommendations</h2>
    <ul>
        {% for rec in recommendations %}
        <li>{{ rec }}</li>
        {% endfor %}
    </ul>
    
    <div style="margin-top: 50px; padding: 20px; background-color: #ecf0f1;">
        <p><small>This report was automatically generated by the ArduCopter Bridge Inspector system.</small></p>
    </div>
</body>
</html>
        """
        
    def generate_report(self, mission_data: Dict[str, Any], defects: List[Dict[str, Any]], 
                       structure_info: Dict[str, Any], output_name: str) -> str:
        """Generate comprehensive inspection report"""
        
        # Create report directory
        report_dir = os.path.join(self.output_dir, output_name)
        os.makedirs(report_dir, exist_ok=True)
        
        # Process inspection data
        summary = self._calculate_summary(mission_data, defects)
        
        # Generate visualizations
        defect_chart_b64 = self._create_defect_chart(summary)
        severity_map_b64 = self._create_severity_heatmap(defects, structure_info)
        
        # Create interactive map
        map_html = self._create_inspection_map(mission_data, defects, structure_info)
        
        # Process defect images
        defects_with_images = self._process_defect_images(defects)
        
        # Generate recommendations
        recommendations = self._generate_recommendations(summary, defects)
        
        # Create HTML report
        html_content = self._render_html_report(
            summary, defects_with_images, structure_info,
            defect_chart_b64, map_html, recommendations
        )
        
        # Save HTML report
        html_path = os.path.join(report_dir, "inspection_report.html")
        with open(html_path, 'w') as f:
            f.write(html_content)
        
        # Generate PDF report
        pdf_path = self._generate_pdf_report(
            report_dir, summary, defects, structure_info, recommendations
        )
        
        # Generate CSV data export
        csv_path = self._export_defects_csv(defects, report_dir)
        
        # Generate JSON summary
        json_path = self._export_json_summary(
            summary, defects, structure_info, recommendations, report_dir
        )
        
        print(f"Report generated successfully:")
        print(f"  HTML: {html_path}")
        print(f"  PDF: {pdf_path}")
        print(f"  CSV: {csv_path}")
        print(f"  JSON: {json_path}")
        
        return html_path
    
    def _calculate_summary(self, mission_data: Dict[str, Any], 
                          defects: List[Dict[str, Any]]) -> InspectionSummary:
        """Calculate inspection summary statistics"""
        
        # Count defects by severity
        severity_counts = {"critical": 0, "severe": 0, "moderate": 0, "minor": 0}
        for defect in defects:
            severity = defect.get("severity", "unknown").lower()
            if severity in severity_counts:
                severity_counts[severity] += 1
        
        # Calculate mission statistics
        waypoints = mission_data.get("waypoints", [])
        completed = mission_data.get("completed_waypoints", len(waypoints))
        
        # Calculate total distance
        total_distance = 0
        for i in range(1, len(waypoints)):
            total_distance += self._calculate_distance(waypoints[i-1], waypoints[i])
        
        summary = InspectionSummary(
            total_waypoints=len(waypoints),
            waypoints_completed=completed,
            total_defects=len(defects),
            critical_defects=severity_counts["critical"],
            severe_defects=severity_counts["severe"],
            moderate_defects=severity_counts["moderate"],
            minor_defects=severity_counts["minor"],
            inspection_duration=mission_data.get("duration", 0) / 60,  # Convert to minutes
            total_distance=total_distance,
            average_battery_used=mission_data.get("battery_used", 0),
            weather_conditions=mission_data.get("weather", {})
        )
        
        return summary
    
    def _create_defect_chart(self, summary: InspectionSummary) -> str:
        """Create defect distribution pie chart"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Pie chart of defect severities
        sizes = [
            summary.critical_defects,
            summary.severe_defects,
            summary.moderate_defects,
            summary.minor_defects
        ]
        labels = ['Critical', 'Severe', 'Moderate', 'Minor']
        colors = ['#e74c3c', '#e67e22', '#f39c12', '#27ae60']
        
        # Remove zero values
        non_zero = [(size, label, color) for size, label, color in 
                   zip(sizes, labels, colors) if size > 0]
        
        if non_zero:
            sizes, labels, colors = zip(*non_zero)
            ax1.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
            ax1.set_title('Defect Severity Distribution')
        else:
            ax1.text(0.5, 0.5, 'No defects detected', ha='center', va='center')
            ax1.set_title('Defect Severity Distribution')
        
        # Bar chart of defect counts
        ax2.bar(range(4), 
               [summary.critical_defects, summary.severe_defects, 
                summary.moderate_defects, summary.minor_defects],
               color=colors)
        ax2.set_xticks(range(4))
        ax2.set_xticklabels(['Critical', 'Severe', 'Moderate', 'Minor'])
        ax2.set_ylabel('Count')
        ax2.set_title('Defect Count by Severity')
        
        # Save to base64
        buffer = BytesIO()
        plt.tight_layout()
        plt.savefig(buffer, format='png', dpi=150)
        buffer.seek(0)
        chart_b64 = base64.b64encode(buffer.getvalue()).decode()
        plt.close()
        
        return chart_b64
    
    def _create_severity_heatmap(self, defects: List[Dict[str, Any]], 
                                structure_info: Dict[str, Any]) -> str:
        """Create heatmap showing defect severity distribution"""
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Extract defect locations and severities
        severity_values = {
            "critical": 4,
            "severe": 3,
            "moderate": 2,
            "minor": 1
        }
        
        x_coords = []
        y_coords = []
        severities = []
        
        for defect in defects:
            if "location" in defect and len(defect["location"]) >= 2:
                x_coords.append(defect["location"][1])  # longitude
                y_coords.append(defect["location"][0])  # latitude
                severities.append(severity_values.get(defect.get("severity", "minor"), 1))
        
        if x_coords:
            # Create scatter plot with severity coloring
            scatter = ax.scatter(x_coords, y_coords, c=severities, s=100, 
                               cmap='YlOrRd', vmin=1, vmax=4, alpha=0.7)
            
            # Add colorbar
            cbar = plt.colorbar(scatter, ax=ax)
            cbar.set_ticks([1, 2, 3, 4])
            cbar.set_ticklabels(['Minor', 'Moderate', 'Severe', 'Critical'])
            cbar.set_label('Defect Severity')
            
            # Add structure outline
            if structure_info:
                center_lon = structure_info.get("center_lon", 0)
                center_lat = structure_info.get("center_lat", 0)
                width = structure_info.get("width", 50) / 111320  # Convert to degrees
                length = structure_info.get("length", 100) / 111320
                
                rect = patches.Rectangle(
                    (center_lon - length/2, center_lat - width/2),
                    length, width,
                    linewidth=2, edgecolor='blue', facecolor='none'
                )
                ax.add_patch(rect)
            
            ax.set_xlabel('Longitude')
            ax.set_ylabel('Latitude')
            ax.set_title('Defect Severity Distribution Map')
        else:
            ax.text(0.5, 0.5, 'No defect locations available', 
                   ha='center', va='center', transform=ax.transAxes)
        
        # Save to base64
        buffer = BytesIO()
        plt.tight_layout()
        plt.savefig(buffer, format='png', dpi=150)
        buffer.seek(0)
        map_b64 = base64.b64encode(buffer.getvalue()).decode()
        plt.close()
        
        return map_b64
    
    def _create_inspection_map(self, mission_data: Dict[str, Any], 
                              defects: List[Dict[str, Any]], 
                              structure_info: Dict[str, Any]) -> str:
        """Create interactive map with inspection path and defects"""
        
        # Get center coordinates
        waypoints = mission_data.get("waypoints", [])
        if waypoints:
            center_lat = waypoints[0]["lat"]
            center_lon = waypoints[0]["lon"]
        else:
            center_lat = structure_info.get("center_lat", 0)
            center_lon = structure_info.get("center_lon", 0)
        
        # Create folium map
        m = folium.Map(location=[center_lat, center_lon], zoom_start=18)
        
        # Add inspection path
        if waypoints:
            path_coords = [[wp["lat"], wp["lon"]] for wp in waypoints]
            folium.PolyLine(
                path_coords,
                color="blue",
                weight=2,
                opacity=0.8,
                popup="Inspection Path"
            ).add_to(m)
            
            # Add waypoint markers
            for i, wp in enumerate(waypoints):
                folium.CircleMarker(
                    [wp["lat"], wp["lon"]],
                    radius=3,
                    popup=f"Waypoint {i+1}: {wp.get('description', '')}",
                    color="blue",
                    fill=True,
                    fillColor="blue"
                ).add_to(m)
        
        # Add defect markers
        severity_colors = {
            "critical": "red",
            "severe": "orange",
            "moderate": "yellow",
            "minor": "green"
        }
        
        for i, defect in enumerate(defects):
            if "location" in defect and len(defect["location"]) >= 2:
                color = severity_colors.get(defect.get("severity", "minor"), "gray")
                
                popup_text = f"""
                <b>Defect {i+1}</b><br>
                Type: {defect.get('type', 'Unknown')}<br>
                Severity: {defect.get('severity', 'Unknown')}<br>
                Confidence: {defect.get('confidence', 0)*100:.1f}%
                """
                
                folium.Marker(
                    [defect["location"][0], defect["location"][1]],
                    popup=popup_text,
                    icon=folium.Icon(color=color, icon='exclamation-sign')
                ).add_to(m)
        
        # Add structure outline
        if structure_info:
            # Simple rectangle representation
            center = [structure_info["center_lat"], structure_info["center_lon"]]
            width_deg = structure_info.get("width", 50) / 111320
            length_deg = structure_info.get("length", 100) / 111320
            
            bounds = [
                [center[0] - width_deg/2, center[1] - length_deg/2],
                [center[0] + width_deg/2, center[1] - length_deg/2],
                [center[0] + width_deg/2, center[1] + length_deg/2],
                [center[0] - width_deg/2, center[1] + length_deg/2],
                [center[0] - width_deg/2, center[1] - length_deg/2]
            ]
            
            folium.PolyLine(
                bounds,
                color="black",
                weight=3,
                opacity=0.8,
                popup=structure_info.get("name", "Structure")
            ).add_to(m)
        
        # Add minimap
        minimap = plugins.MiniMap()
        m.add_child(minimap)
        
        # Get HTML representation
        return m._repr_html_()
    
    def _process_defect_images(self, defects: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Process and encode defect images"""
        processed_defects = []
        
        for defect in defects:
            defect_copy = defect.copy()
            
            if "image_path" in defect and os.path.exists(defect["image_path"]):
                try:
                    # Read and encode image
                    with open(defect["image_path"], "rb") as f:
                        image_data = base64.b64encode(f.read()).decode()
                    defect_copy["image_data"] = image_data
                except Exception as e:
                    print(f"Error processing image {defect['image_path']}: {e}")
                    defect_copy["image_data"] = None
            else:
                defect_copy["image_data"] = None
            
            processed_defects.append(defect_copy)
        
        return processed_defects
    
    def _generate_recommendations(self, summary: InspectionSummary, 
                                 defects: List[Dict[str, Any]]) -> List[str]:
        """Generate maintenance recommendations based on findings"""
        recommendations = []
        
        # Critical defects
        if summary.critical_defects > 0:
            recommendations.append(
                f"URGENT: {summary.critical_defects} critical defects detected. "
                "Immediate structural assessment and repairs recommended."
            )
        
        # Severe defects
        if summary.severe_defects > 0:
            recommendations.append(
                f"{summary.severe_defects} severe defects require attention within 30 days. "
                "Schedule detailed engineering assessment."
            )
        
        # Moderate defects
        if summary.moderate_defects > 0:
            recommendations.append(
                f"{summary.moderate_defects} moderate defects should be monitored. "
                "Plan repairs within 6 months."
            )
        
        # Pattern analysis
        defect_types = {}
        for defect in defects:
            dtype = defect.get("type", "unknown")
            defect_types[dtype] = defect_types.get(dtype, 0) + 1
        
        # Type-specific recommendations
        if defect_types.get("crack", 0) > 5:
            recommendations.append(
                "Multiple cracks detected. Consider comprehensive structural integrity assessment."
            )
        
        if defect_types.get("corrosion", 0) > 3:
            recommendations.append(
                "Significant corrosion present. Implement corrosion protection measures."
            )
        
        if defect_types.get("spalling", 0) > 2:
            recommendations.append(
                "Concrete spalling detected. Evaluate concrete quality and reinforcement coverage."
            )
        
        # General maintenance
        if summary.total_defects == 0:
            recommendations.append(
                "No defects detected. Continue regular inspection schedule."
            )
        else:
            recommendations.append(
                f"Total of {summary.total_defects} defects found. "
                "Update maintenance database and track progression."
            )
        
        # Inspection coverage
        coverage = (summary.waypoints_completed / summary.total_waypoints * 100) if summary.total_waypoints > 0 else 0
        if coverage < 90:
            recommendations.append(
                f"Only {coverage:.1f}% inspection coverage achieved. "
                "Consider re-inspection of missed areas."
            )
        
        return recommendations
    
    def _render_html_report(self, summary: InspectionSummary, 
                           defects: List[Dict[str, Any]], 
                           structure_info: Dict[str, Any],
                           defect_chart_b64: str, 
                           map_html: str,
                           recommendations: List[str]) -> str:
        """Render HTML report using template"""
        template = Template(self.html_template)
        
        return template.render(
            report_date=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            structure_name=structure_info.get("name", "Unknown Structure"),
            summary=summary,
            defects=defects,
            defect_chart=defect_chart_b64,
            map_html=map_html,
            recommendations=recommendations
        )
    
    def _generate_pdf_report(self, report_dir: str, summary: InspectionSummary,
                           defects: List[Dict[str, Any]], 
                           structure_info: Dict[str, Any],
                           recommendations: List[str]) -> str:
        """Generate PDF version of report"""
        pdf_path = os.path.join(report_dir, "inspection_report.pdf")
        
        with PdfPages(pdf_path) as pdf:
            # Title page
            fig, ax = plt.subplots(figsize=(8.5, 11))
            ax.axis('off')
            
            # Title
            ax.text(0.5, 0.9, 'Structure Inspection Report', 
                   ha='center', size=24, weight='bold')
            ax.text(0.5, 0.85, structure_info.get("name", "Unknown Structure"), 
                   ha='center', size=18)
            ax.text(0.5, 0.8, datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 
                   ha='center', size=14)
            
            # Summary box
            summary_text = f"""
Inspection Summary:
• Total Defects: {summary.total_defects}
• Critical: {summary.critical_defects}
• Severe: {summary.severe_defects}
• Moderate: {summary.moderate_defects}
• Minor: {summary.minor_defects}

Mission Statistics:
• Duration: {summary.inspection_duration:.1f} minutes
• Distance: {summary.total_distance:.1f} meters
• Coverage: {summary.waypoints_completed}/{summary.total_waypoints} waypoints
            """
            
            ax.text(0.1, 0.6, summary_text, va='top', size=12, 
                   family='monospace', wrap=True)
            
            pdf.savefig(fig)
            plt.close()
            
            # Defect summary page
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8.5, 11))
            
            # Defect table
            if defects:
                defect_data = []
                for i, defect in enumerate(defects[:20]):  # First 20 defects
                    defect_data.append([
                        i+1,
                        defect.get("type", "Unknown"),
                        defect.get("severity", "Unknown"),
                        f"{defect.get('confidence', 0)*100:.1f}%"
                    ])
                
                table = ax1.table(
                    cellText=defect_data,
                    colLabels=["ID", "Type", "Severity", "Confidence"],
                    cellLoc='left',
                    loc='center'
                )
                table.auto_set_font_size(False)
                table.set_fontsize(10)
                ax1.axis('off')
                ax1.set_title("Detected Defects", size=16, weight='bold', pad=20)
            
            # Recommendations
            ax2.axis('off')
            ax2.text(0.5, 0.9, "Recommendations", ha='center', size=16, weight='bold')
            
            rec_text = "\n".join([f"• {rec}" for rec in recommendations])
            ax2.text(0.1, 0.8, rec_text, va='top', size=11, wrap=True)
            
            pdf.savefig(fig)
            plt.close()
        
        return pdf_path
    
    def _export_defects_csv(self, defects: List[Dict[str, Any]], report_dir: str) -> str:
        """Export defects to CSV file"""
        csv_path = os.path.join(report_dir, "defects.csv")
        
        # Prepare data for DataFrame
        rows = []
        for i, defect in enumerate(defects):
            row = {
                "id": i + 1,
                "type": defect.get("type", ""),
                "severity": defect.get("severity", ""),
                "confidence": defect.get("confidence", 0),
                "latitude": defect.get("location", [0, 0])[0] if "location" in defect else 0,
                "longitude": defect.get("location", [0, 0])[1] if "location" in defect and len(defect["location"]) > 1 else 0,
                "width_mm": defect.get("dimensions", {}).get("width", 0) if "dimensions" in defect else 0,
                "length_mm": defect.get("dimensions", {}).get("length", 0) if "dimensions" in defect else 0,
                "area_mm2": defect.get("area", 0),
                "image_path": defect.get("image_path", ""),
                "notes": defect.get("notes", "")
            }
            rows.append(row)
        
        df = pd.DataFrame(rows)
        df.to_csv(csv_path, index=False)
        
        return csv_path
    
    def _export_json_summary(self, summary: InspectionSummary, 
                           defects: List[Dict[str, Any]], 
                           structure_info: Dict[str, Any],
                           recommendations: List[str],
                           report_dir: str) -> str:
        """Export complete inspection data as JSON"""
        json_path = os.path.join(report_dir, "inspection_data.json")
        
        data = {
            "report_generated": datetime.now().isoformat(),
            "structure": structure_info,
            "summary": asdict(summary),
            "defects": defects,
            "recommendations": recommendations,
            "metadata": {
                "report_version": "1.0",
                "system": "ArduCopter Bridge Inspector"
            }
        }
        
        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2)
        
        return json_path
    
    def _calculate_distance(self, p1: Dict, p2: Dict) -> float:
        """Calculate distance between two waypoints"""
        lat1, lon1 = p1["lat"], p1["lon"]
        lat2, lon2 = p2["lat"], p2["lon"]
        
        dlat = (lat2 - lat1) * 111320
        dlon = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
        
        return math.sqrt(dlat**2 + dlon**2)


def main():
    """Test report generation"""
    generator = ReportGenerator()
    
    # Sample data
    mission_data = {
        "waypoints": [
            {"lat": 37.8199, "lon": -122.4783, "alt": 30},
            {"lat": 37.8201, "lon": -122.4785, "alt": 40},
        ],
        "completed_waypoints": 2,
        "duration": 1200,  # seconds
        "battery_used": 35,
        "weather": {"wind_speed": 5, "temperature": 22}
    }
    
    defects = [
        {
            "type": "crack",
            "severity": "severe",
            "confidence": 0.85,
            "location": [37.8200, -122.4784],
            "dimensions": {"width": 2.5, "length": 150},
            "area": 375,
            "notes": "Longitudinal crack on deck surface"
        },
        {
            "type": "spalling",
            "severity": "moderate",
            "confidence": 0.72,
            "location": [37.8199, -122.4782],
            "dimensions": {"width": 50, "length": 80},
            "area": 4000,
            "notes": "Concrete spalling on pier"
        }
    ]
    
    structure_info = {
        "name": "Golden Gate Bridge - North Section",
        "type": "bridge",
        "center_lat": 37.8199,
        "center_lon": -122.4783,
        "width": 27,
        "length": 200,
        "height": 227
    }
    
    # Generate report
    report_path = generator.generate_report(
        mission_data, defects, structure_info, 
        f"inspection_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    
    print(f"Report generated: {report_path}")


if __name__ == "__main__":
    main()