"""UAV Strategic Deconfliction System

A comprehensive system for detecting and resolving conflicts in shared airspace.
Generates realistic, smooth UAV trajectories natively with built-in conflict detection
and resolution strategies.
"""

__version__ = "1.0.0"
__author__ = "FlytBase Assignment 2025"

from .flight_plan import FlightPlan, Waypoint
from .data_generator import FlightDataGenerator
from .conflict_detector import ConflictDetector, Conflict
from .visualization import FlightVisualizer, create_quick_animation
from .verification_service import MissionVerificationService, VerificationResult, create_verification_service_from_file
from .edge_case_generator import EdgeCaseGenerator

__all__ = [
    'FlightPlan',
    'Waypoint',
    'FlightDataGenerator',
    'ConflictDetector',
    'Conflict',
    'FlightVisualizer',
    'create_quick_animation',
    'MissionVerificationService',
    'VerificationResult',
    'create_verification_service_from_file',
    'EdgeCaseGenerator',
]
