"""
Visualization utilities for UAV flight plans and conflicts.

This module provides functions to visualize flight paths, conflicts, and resolutions.
"""

from typing import List, Optional
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

from .flight_plan import FlightPlan
from .conflict_detector import Conflict


class FlightVisualizer:
    """
    Provides visualization capabilities for UAV scenarios.
    """
    
    def __init__(self, figsize: tuple = (12, 8), dpi: int = 100):
        """
        Initialize the visualizer.
        
        Args:
            figsize: Figure size for plots
            dpi: Dots per inch for rendering quality
        """
        self.figsize = figsize
        self.dpi = dpi
        self.colors = plt.cm.tab10(np.linspace(0, 1, 10))
    
    def plot_flights_3d(
        self,
        flights: List[FlightPlan],
        conflicts: Optional[List[Conflict]] = None,
        show_waypoints: bool = False,
        title: str = "UAV Flight Paths"
    ):
        """
        Plot flight paths in 3D space.
        
        Args:
            flights: List of flight plans to plot (already smooth if generated with default settings)
            conflicts: Optional list of conflicts to visualize
            show_waypoints: Whether to show waypoint markers (usually False for smooth paths)
            title: Plot title
            
        Returns:
            fig: The matplotlib figure object
        """
        fig = plt.figure(figsize=self.figsize, dpi=self.dpi)
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot each flight
        linestyles = ['-', '--', '-.', ':']  # Different line styles for visibility
        for i, flight in enumerate(flights):
            color = self.colors[i % len(self.colors)]
            linestyle = linestyles[i % len(linestyles)]
            linewidth = 3 if i == 0 else 2  # Make first flight (primary) thicker
            
            # Extract coordinates
            xs = [wp.x for wp in flight.waypoints]
            ys = [wp.y for wp in flight.waypoints]
            zs = [wp.z for wp in flight.waypoints]
            
            # Plot path
            ax.plot(xs, ys, zs, color=color, linewidth=linewidth,
                   linestyle=linestyle, label=f"{flight.uav_id} (P{flight.priority})", alpha=0.8)
            
            # Plot waypoints
            if show_waypoints:
                ax.scatter(xs, ys, zs, color=color, s=50, marker='o', alpha=0.8)
                
                # Mark start and end
                ax.scatter([xs[0]], [ys[0]], [zs[0]], color=color, s=200,
                          marker='^', edgecolors='black', linewidths=2)
                ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], color=color, s=200,
                          marker='s', edgecolors='black', linewidths=2)
        
        # Plot conflicts
        if conflicts:
            for conflict in conflicts:
                pos1 = conflict.position1
                pos2 = conflict.position2
                
                # Draw line between conflict positions
                ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]],
                       'r--', linewidth=2, alpha=0.6)
                
                # Mark conflict points
                ax.scatter([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]],
                          color='red', s=150, marker='X', edgecolors='black',
                          linewidths=2, zorder=10)
        
        ax.set_xlabel('X (meters)', fontsize=10)
        ax.set_ylabel('Y (meters)', fontsize=10)
        ax.set_zlabel('Altitude (meters)', fontsize=10)
        ax.set_title(title, fontsize=14, fontweight='bold')
        
        # Legend
        ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), fontsize=8)
        
        plt.tight_layout()
        return fig
    
    def plot_flights_2d_top(
        self,
        flights: List[FlightPlan],
        conflicts: Optional[List[Conflict]] = None,
        title: str = "UAV Flight Paths (Top View)"
    ):
        """
        Plot flight paths from top view (X-Y plane).
        
        Args:
            flights: List of flight plans to plot
            conflicts: Optional list of conflicts to visualize
            title: Plot title
            
        Returns:
            fig: The matplotlib figure object
        """
        fig, ax = plt.subplots(figsize=self.figsize, dpi=self.dpi)
        
        # Plot each flight
        for i, flight in enumerate(flights):
            color = self.colors[i % len(self.colors)]
            
            xs = [wp.x for wp in flight.waypoints]
            ys = [wp.y for wp in flight.waypoints]
            
            ax.plot(xs, ys, color=color, linewidth=2,
                   label=f"{flight.uav_id}", alpha=0.7)
            ax.scatter(xs, ys, color=color, s=50, alpha=0.8)
            
            # Mark start
            ax.scatter([xs[0]], [ys[0]], color=color, s=200,
                      marker='^', edgecolors='black', linewidths=2)
        
        # Plot conflicts
        if conflicts:
            for conflict in conflicts:
                pos1 = conflict.position1
                pos2 = conflict.position2
                
                ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]],
                       'r--', linewidth=2, alpha=0.6)
                ax.scatter([pos1[0], pos2[0]], [pos1[1], pos2[1]],
                          color='red', s=150, marker='X',
                          edgecolors='black', linewidths=2, zorder=10)
        
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        plt.tight_layout()
        return fig
    
    def plot_altitude_profile(
        self,
        flights: List[FlightPlan],
        conflicts: Optional[List[Conflict]] = None,
        title: str = "Altitude Profiles"
    ):
        """
        Plot altitude vs time for all flights.
        
        Args:
            flights: List of flight plans to plot
            conflicts: Optional list of conflicts to visualize
            title: Plot title
        """
        fig, ax = plt.subplots(figsize=self.figsize)
        
        # Plot each flight
        for i, flight in enumerate(flights):
            color = self.colors[i % len(self.colors)]
            
            times = [wp.time for wp in flight.waypoints]
            alts = [wp.z for wp in flight.waypoints]
            
            ax.plot(times, alts, color=color, linewidth=2,
                   label=f"{flight.uav_id}", alpha=0.7, marker='o')
        
        # Mark conflict times
        if conflicts:
            conflict_times = [c.time for c in conflicts]
            ymin, ymax = ax.get_ylim()
            
            for t in conflict_times:
                ax.axvline(x=t, color='red', linestyle='--',
                          linewidth=1.5, alpha=0.5)
        
        ax.set_xlabel('Time (seconds)', fontsize=12)
        ax.set_ylabel('Altitude (meters)', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def plot_comparison(
        self,
        original_flights: List[FlightPlan],
        resolved_flights: List[FlightPlan],
        original_conflicts: List[Conflict],
        resolved_conflicts: List[Conflict]
    ):
        """
        Plot side-by-side comparison of original and resolved scenarios.
        
        Args:
            original_flights: Original flight plans
            resolved_flights: Resolved flight plans
            original_conflicts: Conflicts in original scenario
            resolved_conflicts: Conflicts in resolved scenario
        """
        fig = plt.figure(figsize=(16, 7))
        
        # Original scenario
        ax1 = fig.add_subplot(121, projection='3d')
        self._plot_3d_on_axis(ax1, original_flights, original_conflicts,
                              f"Original ({len(original_conflicts)} conflicts)")
        
        # Resolved scenario
        ax2 = fig.add_subplot(122, projection='3d')
        self._plot_3d_on_axis(ax2, resolved_flights, resolved_conflicts,
                              f"Resolved ({len(resolved_conflicts)} conflicts)")
        
        plt.tight_layout()
        return fig
    
    def _plot_3d_on_axis(
        self,
        ax,
        flights: List[FlightPlan],
        conflicts: List[Conflict],
        title: str
    ):
        """Helper method to plot 3D on a specific axis."""
        for i, flight in enumerate(flights):
            color = self.colors[i % len(self.colors)]
            
            xs = [wp.x for wp in flight.waypoints]
            ys = [wp.y for wp in flight.waypoints]
            zs = [wp.z for wp in flight.waypoints]
            
            ax.plot(xs, ys, zs, color=color, linewidth=2, alpha=0.7)
            ax.scatter([xs[0]], [ys[0]], [zs[0]], color=color, s=100,
                      marker='^', edgecolors='black', linewidths=1)
        
        # Plot conflicts
        if conflicts:
            for conflict in conflicts:
                pos1 = conflict.position1
                pos2 = conflict.position2
                
                ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]],
                       'r--', linewidth=1.5, alpha=0.6)
                ax.scatter([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]],
                          color='red', s=100, marker='X', edgecolors='black',
                          linewidths=1, zorder=10)
        
        ax.set_xlabel('X (m)', fontsize=9)
        ax.set_ylabel('Y (m)', fontsize=9)
        ax.set_zlabel('Altitude (m)', fontsize=9)
        ax.set_title(title, fontsize=12, fontweight='bold')
    
    def plot_statistics(
        self,
        conflicts: List[Conflict],
        title: str = "Conflict Statistics"
    ):
        """
        Plot statistics about conflicts.
        
        Args:
            conflicts: List of conflicts
            title: Plot title
        """
        if not conflicts:
            print("No conflicts to plot")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=self.figsize)
        
        # Distance distribution
        distances = [c.distance for c in conflicts]
        axes[0, 0].hist(distances, bins=20, edgecolor='black', alpha=0.7)
        axes[0, 0].set_xlabel('Distance (meters)')
        axes[0, 0].set_ylabel('Frequency')
        axes[0, 0].set_title('Conflict Distance Distribution')
        axes[0, 0].grid(True, alpha=0.3)
        
        # Severity distribution
        severities = [c.severity for c in conflicts]
        axes[0, 1].hist(severities, bins=20, edgecolor='black', alpha=0.7, color='orange')
        axes[0, 1].set_xlabel('Severity (0-1)')
        axes[0, 1].set_ylabel('Frequency')
        axes[0, 1].set_title('Conflict Severity Distribution')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Time distribution
        times = [c.time for c in conflicts]
        axes[1, 0].hist(times, bins=20, edgecolor='black', alpha=0.7, color='green')
        axes[1, 0].set_xlabel('Time (seconds)')
        axes[1, 0].set_ylabel('Frequency')
        axes[1, 0].set_title('Conflict Time Distribution')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Summary statistics
        stats_text = f"""
        Total Conflicts: {len(conflicts)}
        
        Distance:
          Min: {min(distances):.1f} m
          Max: {max(distances):.1f} m
          Mean: {np.mean(distances):.1f} m
        
        Severity:
          Min: {min(severities):.2f}
          Max: {max(severities):.2f}
          Mean: {np.mean(severities):.2f}
        
        Critical (>0.8): {sum(1 for s in severities if s > 0.8)}
        """
        
        axes[1, 1].text(0.1, 0.5, stats_text, fontsize=10,
                       verticalalignment='center', fontfamily='monospace')
        axes[1, 1].axis('off')
        
        fig.suptitle(title, fontsize=14, fontweight='bold')
        plt.tight_layout()
        return fig
    
    # ========================================================================
    # ANIMATION METHODS - Time-based animated path tracing
    # ========================================================================
    
    def animate_3d(
        self,
        flights: List[FlightPlan],
        conflicts: Optional[List[Conflict]] = None,
        duration: float = 15.0,
        fps: int = 30,
        trail_length: int = 80,
        show_full_paths: bool = True,
        rotate_view: bool = True,
        show_time: bool = True,
        keep_traced_path: bool = True
    ) -> animation.FuncAnimation:
        """
        Create 3D animation with drones tracing their paths.
        
        Args:
            flights: List of flight plans to animate
            conflicts: Optional conflicts to highlight when they occur
            duration: Animation duration in seconds
            fps: Frames per second (higher = smoother but larger)
            trail_length: Number of recent positions to show in bright trail
            show_full_paths: Whether to show complete flight paths (faded background)
            rotate_view: Auto-rotate camera for better perspective
            show_time: Show current simulation time
            keep_traced_path: Keep the traced path visible (permanent drawing)
            
        Returns:
            Animation object (use .save() or display in notebook)
        """
        from matplotlib import animation
        from matplotlib.patches import Circle
        
        # Setup
        fig = plt.figure(figsize=(14, 10), dpi=100)
        ax = fig.add_subplot(111, projection='3d')
        
        # Calculate time bounds
        all_times = []
        for flight in flights:
            all_times.extend([wp.time for wp in flight.waypoints])
        
        min_time = min(all_times)
        max_time = max(all_times)
        total_flight_time = max_time - min_time
        
        # Calculate spatial bounds
        all_positions = []
        for flight in flights:
            for wp in flight.waypoints:
                all_positions.append([wp.x, wp.y, wp.z])
        all_positions = np.array(all_positions)
        
        # Set axis limits with margins
        margin = 500
        ax.set_xlim(all_positions[:, 0].min() - margin, all_positions[:, 0].max() + margin)
        ax.set_ylim(all_positions[:, 1].min() - margin, all_positions[:, 1].max() + margin)
        ax.set_zlim(all_positions[:, 2].min() - 50, all_positions[:, 2].max() + 50)
        
        # Styling
        ax.set_xlabel('X (m)', fontsize=10, fontweight='bold')
        ax.set_ylabel('Y (m)', fontsize=10, fontweight='bold')
        ax.set_zlabel('Altitude (m)', fontsize=10, fontweight='bold')
        ax.set_title('UAV Flight Animation - Real-time Path Tracing', 
                    fontsize=14, fontweight='bold', pad=20)
        
        # Background color
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.grid(True, alpha=0.3)
        
        # Color scheme
        drone_colors = plt.cm.tab10(np.linspace(0, 1, 10))
        
        # Plot complete paths (faded background)
        path_lines = []
        if show_full_paths:
            for i, flight in enumerate(flights):
                color = drone_colors[i % len(drone_colors)]
                xs = [wp.x for wp in flight.waypoints]
                ys = [wp.y for wp in flight.waypoints]
                zs = [wp.z for wp in flight.waypoints]
                
                line, = ax.plot(xs, ys, zs, color=color, alpha=0.15,
                              linewidth=1, linestyle='--', zorder=1)
                path_lines.append(line)
        
        # Initialize drone markers (larger, more visible)
        drone_markers = []
        for i, flight in enumerate(flights):
            color = drone_colors[i % len(drone_colors)]
            marker = ax.scatter([], [], [], c=[color], s=200, marker='o',
                              edgecolors='black', linewidths=2, zorder=10,
                              label=f'{flight.uav_id}')
            drone_markers.append(marker)
        
        # Initialize permanent traced paths (what the drone has flown)
        traced_paths = []
        traced_data = []
        if keep_traced_path:
            for i, flight in enumerate(flights):
                color = drone_colors[i % len(drone_colors)]
                line, = ax.plot([], [], [], color=color, linewidth=2,
                              alpha=0.5, zorder=3)  # Permanent but subtle
                traced_paths.append(line)
                traced_data.append({'xs': [], 'ys': [], 'zs': []})
        
        # Initialize recent trail lines (bright, shows current movement)
        trail_lines = []
        trail_data = []
        for i, flight in enumerate(flights):
            color = drone_colors[i % len(drone_colors)]
            line, = ax.plot([], [], [], color=color, linewidth=3.5,
                          alpha=0.9, zorder=5)  # Brighter for recent trail
            trail_lines.append(line)
            trail_data.append({'xs': [], 'ys': [], 'zs': []})
        
        # Conflict markers (will appear when conflicts are active)
        conflict_markers = []
        if conflicts:
            for conflict in conflicts:
                # Create invisible marker initially
                marker = ax.scatter([], [], [], c='red', s=300, marker='X',
                                  edgecolors='black', linewidths=3, zorder=15)
                conflict_markers.append((conflict, marker))
        
        # Time display text
        time_text = None
        if show_time:
            time_text = ax.text2D(0.02, 0.98, '', transform=ax.transAxes,
                                fontsize=12, fontweight='bold',
                                verticalalignment='top',
                                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Add legend
        ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), fontsize=9,
                 framealpha=0.9, title='Drones', title_fontsize=10)
        
        # Animation parameters
        num_frames = int(duration * fps)
        
        # Initial view angle
        ax.view_init(elev=25, azim=45)
        
        def init():
            """Initialize animation."""
            for marker in drone_markers:
                marker._offsets3d = ([], [], [])
            if keep_traced_path:
                for line in traced_paths:
                    line.set_data([], [])
                    line.set_3d_properties([])
            for line in trail_lines:
                line.set_data([], [])
                line.set_3d_properties([])
            for _, marker in conflict_markers:
                marker._offsets3d = ([], [], [])
            if time_text:
                time_text.set_text('')
            
            artists = drone_markers + trail_lines + [m for _, m in conflict_markers]
            if keep_traced_path:
                artists += traced_paths
            if time_text:
                artists.append(time_text)
            return artists
        
        def animate_frame(frame):
            """Update animation for each frame."""
            # Calculate current simulation time
            progress = frame / num_frames
            current_time = min_time + progress * total_flight_time
            
            # Update each drone
            for i, flight in enumerate(flights):
                pos = flight.interpolate_position(current_time)
                
                if pos is not None:
                    # Update drone marker position
                    drone_markers[i]._offsets3d = ([pos[0]], [pos[1]], [pos[2]])
                    
                    # Add to permanent traced path (keeps growing)
                    if keep_traced_path:
                        traced_data[i]['xs'].append(pos[0])
                        traced_data[i]['ys'].append(pos[1])
                        traced_data[i]['zs'].append(pos[2])
                        
                        # Update permanent traced path (all positions)
                        traced_paths[i].set_data(traced_data[i]['xs'], traced_data[i]['ys'])
                        traced_paths[i].set_3d_properties(traced_data[i]['zs'])
                    
                    # Add to recent trail (limited length for emphasis)
                    trail_data[i]['xs'].append(pos[0])
                    trail_data[i]['ys'].append(pos[1])
                    trail_data[i]['zs'].append(pos[2])
                    
                    # Limit trail length (only recent positions shown bright)
                    if len(trail_data[i]['xs']) > trail_length:
                        trail_data[i]['xs'] = trail_data[i]['xs'][-trail_length:]
                        trail_data[i]['ys'] = trail_data[i]['ys'][-trail_length:]
                        trail_data[i]['zs'] = trail_data[i]['zs'][-trail_length:]
                    
                    # Update recent trail line (bright)
                    trail_lines[i].set_data(trail_data[i]['xs'], trail_data[i]['ys'])
                    trail_lines[i].set_3d_properties(trail_data[i]['zs'])
                else:
                    # Drone not active at this time
                    drone_markers[i]._offsets3d = ([], [], [])
            
            # Update conflict markers (show only when conflict is active)
            for conflict, marker in conflict_markers:
                # Show conflict if current time is within 5 seconds of conflict time
                time_tolerance = 5.0
                if abs(current_time - conflict.time) <= time_tolerance:
                    # Conflict is active - show it
                    pos1 = conflict.position1
                    marker._offsets3d = ([pos1[0]], [pos1[1]], [pos1[2]])
                else:
                    # Conflict not active - hide it
                    marker._offsets3d = ([], [], [])
            
            # Update time display
            if time_text:
                time_text.set_text(f'Time: {current_time:.1f}s / {max_time:.1f}s\n'
                                 f'Progress: {progress*100:.0f}%')
            
            # Rotate view for better perspective
            if rotate_view:
                azim = 45 + frame * 0.3  # Slow rotation
                ax.view_init(elev=25, azim=azim)
            
            artists = drone_markers + trail_lines + [m for _, m in conflict_markers]
            if keep_traced_path:
                artists += traced_paths
            if time_text:
                artists.append(time_text)
            return artists
        
        # Create animation
        anim = animation.FuncAnimation(
            fig, animate_frame, init_func=init,
            frames=num_frames, interval=1000/fps,
            blit=False, repeat=True
        )
        
        plt.tight_layout()
        return anim
    
    def animate_2d_topdown(
        self,
        flights: List[FlightPlan],
        conflicts: Optional[List[Conflict]] = None,
        duration: float = 15.0,
        fps: int = 30,
        trail_length: int = 100,
        show_altitude_colors: bool = True,
        keep_traced_path: bool = True
    ) -> animation.FuncAnimation:
        """
        Create top-down 2D animation with altitude shown as color.
        
        Args:
            flights: List of flight plans to animate
            conflicts: Optional conflicts to highlight
            duration: Animation duration in seconds
            fps: Frames per second
            trail_length: Number of recent positions to show in bright trail
            show_altitude_colors: Color trails by altitude
            keep_traced_path: Keep the traced path visible (permanent drawing)
            
        Returns:
            Animation object
        """
        from matplotlib import animation
        from matplotlib.patches import Circle
        
        fig, ax = plt.subplots(figsize=(14, 10), dpi=100)
        
        # Calculate time bounds
        all_times = []
        for flight in flights:
            all_times.extend([wp.time for wp in flight.waypoints])
        
        min_time = min(all_times)
        max_time = max(all_times)
        total_flight_time = max_time - min_time
        
        # Calculate spatial bounds
        all_positions = []
        for flight in flights:
            for wp in flight.waypoints:
                all_positions.append([wp.x, wp.y, wp.z])
        all_positions = np.array(all_positions)
        
        # Set axis limits
        margin = 500
        ax.set_xlim(all_positions[:, 0].min() - margin, all_positions[:, 0].max() + margin)
        ax.set_ylim(all_positions[:, 1].min() - margin, all_positions[:, 1].max() + margin)
        
        # Styling
        ax.set_xlabel('X (m)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
        ax.set_title('UAV Flight Animation - Top-Down View', 
                    fontsize=14, fontweight='bold')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_facecolor('#f0f0f0')
        
        # Color scheme
        drone_colors = plt.cm.tab10(np.linspace(0, 1, 10))
        
        # Plot complete paths (faded)
        for i, flight in enumerate(flights):
            color = drone_colors[i % len(drone_colors)]
            xs = [wp.x for wp in flight.waypoints]
            ys = [wp.y for wp in flight.waypoints]
            ax.plot(xs, ys, color=color, alpha=0.2, linewidth=1, linestyle='--')
        
        # Initialize drone markers
        drone_markers = []
        drone_labels = []
        for i, flight in enumerate(flights):
            color = drone_colors[i % len(drone_colors)]
            marker, = ax.plot([], [], 'o', color=color, markersize=15,
                            markeredgecolor='black', markeredgewidth=2,
                            label=f'{flight.uav_id}', zorder=10)
            drone_markers.append(marker)
            
            # Add text label
            label = ax.text(0, 0, flight.uav_id, fontsize=8, fontweight='bold',
                          ha='center', va='bottom', zorder=11)
            label.set_visible(False)
            drone_labels.append(label)
        
        # Initialize permanent traced paths
        traced_paths = []
        traced_data = []
        if keep_traced_path:
            for i, flight in enumerate(flights):
                color = drone_colors[i % len(drone_colors)]
                line, = ax.plot([], [], color=color, linewidth=2,
                              alpha=0.4, zorder=3)  # Permanent but subtle
                traced_paths.append(line)
                traced_data.append({'xs': [], 'ys': [], 'zs': []})
        
        # Initialize recent trail lines (bright)
        trail_lines = []
        trail_data = []
        for i, flight in enumerate(flights):
            color = drone_colors[i % len(drone_colors)]
            line, = ax.plot([], [], color=color, linewidth=3.5,
                          alpha=0.85, zorder=5)  # Brighter for recent
            trail_lines.append(line)
            trail_data.append({'xs': [], 'ys': [], 'zs': []})
        
        # Conflict circles
        conflict_circles = []
        if conflicts:
            for conflict in conflicts:
                circle = Circle((0, 0), 60, color='red', alpha=0.3, zorder=3)
                circle.set_visible(False)
                ax.add_patch(circle)
                conflict_circles.append((conflict, circle))
        
        # Time display
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                          fontsize=12, fontweight='bold',
                          verticalalignment='top',
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
        
        # Legend
        ax.legend(loc='upper right', fontsize=9, framealpha=0.9,
                 title='Drones', title_fontsize=10)
        
        # Animation parameters
        num_frames = int(duration * fps)
        
        def init():
            """Initialize animation."""
            for marker in drone_markers:
                marker.set_data([], [])
            for label in drone_labels:
                label.set_visible(False)
            if keep_traced_path:
                for line in traced_paths:
                    line.set_data([], [])
            for line in trail_lines:
                line.set_data([], [])
            for _, circle in conflict_circles:
                circle.set_visible(False)
            time_text.set_text('')
            
            artists = drone_markers + drone_labels + trail_lines + [c for _, c in conflict_circles] + [time_text]
            if keep_traced_path:
                artists += traced_paths
            return artists
        
        def animate_frame(frame):
            """Update animation for each frame."""
            progress = frame / num_frames
            current_time = min_time + progress * total_flight_time
            
            # Update each drone
            for i, flight in enumerate(flights):
                pos = flight.interpolate_position(current_time)
                
                if pos is not None:
                    # Update marker
                    drone_markers[i].set_data([pos[0]], [pos[1]])
                    
                    # Update label
                    drone_labels[i].set_position((pos[0], pos[1] + 150))
                    drone_labels[i].set_visible(True)
                    
                    # Add to permanent traced path (keeps growing)
                    if keep_traced_path:
                        traced_data[i]['xs'].append(pos[0])
                        traced_data[i]['ys'].append(pos[1])
                        traced_data[i]['zs'].append(pos[2])
                        
                        # Update permanent traced path
                        traced_paths[i].set_data(traced_data[i]['xs'], traced_data[i]['ys'])
                    
                    # Add to recent trail (limited length)
                    trail_data[i]['xs'].append(pos[0])
                    trail_data[i]['ys'].append(pos[1])
                    trail_data[i]['zs'].append(pos[2])
                    
                    # Limit trail length
                    if len(trail_data[i]['xs']) > trail_length:
                        trail_data[i]['xs'] = trail_data[i]['xs'][-trail_length:]
                        trail_data[i]['ys'] = trail_data[i]['ys'][-trail_length:]
                        trail_data[i]['zs'] = trail_data[i]['zs'][-trail_length:]
                    
                    # Update recent trail (bright)
                    trail_lines[i].set_data(trail_data[i]['xs'], trail_data[i]['ys'])
                else:
                    drone_markers[i].set_data([], [])
                    drone_labels[i].set_visible(False)
            
            # Update conflicts
            for conflict, circle in conflict_circles:
                # Show conflict if current time is within 5 seconds of conflict time
                time_tolerance = 5.0
                if abs(current_time - conflict.time) <= time_tolerance:
                    pos1 = conflict.position1
                    circle.center = (pos1[0], pos1[1])
                    circle.set_visible(True)
                else:
                    circle.set_visible(False)
            
            # Update time
            time_text.set_text(f'Time: {current_time:.1f}s\nProgress: {progress*100:.0f}%')
            
            artists = drone_markers + drone_labels + trail_lines + [c for _, c in conflict_circles] + [time_text]
            if keep_traced_path:
                artists += traced_paths
            return artists
        
        # Create animation
        anim = animation.FuncAnimation(
            fig, animate_frame, init_func=init,
            frames=num_frames, interval=1000/fps,
            blit=True, repeat=True
        )
        
        plt.tight_layout()
        return anim
    
    def animate_altitude_profile(
        self,
        flights: List[FlightPlan],
        conflicts: Optional[List[Conflict]] = None,
        duration: float = 10,
        fps: int = 30
    ):
        """
        Create animated altitude profile showing temporal evolution.
        
        Args:
            flights: List of flight plans
            conflicts: Optional list of conflicts
            duration: Duration of animation in seconds
            fps: Frames per second
            
        Returns:
            Animation object
        """
        fig, ax = plt.subplots(figsize=self.figsize, dpi=self.dpi)
        
        # Find time and altitude ranges
        all_times = []
        all_altitudes = []
        for flight in flights:
            times = [wp.time for wp in flight.waypoints]
            altitudes = [wp.z for wp in flight.waypoints]
            all_times.extend(times)
            all_altitudes.extend(altitudes)
        
        min_time = min(all_times)
        max_time = max(all_times)
        min_alt = min(all_altitudes) - 20
        max_alt = max(all_altitudes) + 20
        
        # Prepare full flight paths (will be shown progressively)
        flight_data = []
        for i, flight in enumerate(flights):
            color = self.colors[i % len(self.colors)]
            times = [wp.time for wp in flight.waypoints]
            altitudes = [wp.z for wp in flight.waypoints]
            flight_data.append({
                'times': times,
                'altitudes': altitudes,
                'color': color,
                'label': flight.uav_id
            })
        
        # Animation function
        def animate_frame(frame_num):
            ax.clear()
            
            # Current time in simulation
            current_time = min_time + (max_time - min_time) * frame_num / (fps * duration)
            
            # Plot each flight's altitude up to current time
            for data in flight_data:
                # Find indices up to current time
                visible_indices = [i for i, t in enumerate(data['times']) if t <= current_time]
                
                if visible_indices:
                    visible_times = [data['times'][i] for i in visible_indices]
                    visible_alts = [data['altitudes'][i] for i in visible_indices]
                    
                    # Plot the visible portion
                    ax.plot(visible_times, visible_alts, 
                           color=data['color'], linewidth=2, 
                           label=data['label'], alpha=0.8)
                    
                    # Mark current position
                    if len(visible_times) > 0:
                        ax.scatter([visible_times[-1]], [visible_alts[-1]], 
                                 color=data['color'], s=100, zorder=5,
                                 edgecolors='black', linewidths=1.5)
            
            # Show conflict times if any occur up to current time
            if conflicts:
                conflict_times = [c.time for c in conflicts if c.time <= current_time]
                for t in conflict_times:
                    ax.axvline(x=t, color='red', linestyle='--',
                             linewidth=2, alpha=0.6, zorder=1)
            
            # Time indicator
            ax.axvline(x=current_time, color='green', linestyle='-',
                      linewidth=2, alpha=0.5, label=f'Current Time: {current_time:.1f}s')
            
            ax.set_xlim(min_time, max_time)
            ax.set_ylim(min_alt, max_alt)
            ax.set_xlabel('Time (seconds)', fontsize=12)
            ax.set_ylabel('Altitude (meters)', fontsize=12)
            ax.set_title(f'Altitude Profile Animation (t={current_time:.1f}s)', 
                        fontsize=14, fontweight='bold')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
        
        # Create animation
        total_frames = int(fps * duration)
        anim = animation.FuncAnimation(
            fig, animate_frame, frames=total_frames,
            interval=1000/fps, blit=False, repeat=True
        )
        
        return anim
    
    def save_animation(
        self,
        anim: animation.FuncAnimation,
        filename: str,
        writer: str = 'ffmpeg',
        fps: int = 30,
        dpi: int = 100,
        bitrate: int = 1800
    ):
        """
        Save animation to file.
        
        Args:
            anim: Animation object to save
            filename: Output filename (.mp4, .gif, etc.)
            writer: Writer to use ('ffmpeg', 'pillow', 'imagemagick')
            fps: Frames per second
            dpi: Resolution (higher = better quality)
            bitrate: Video bitrate (higher = better quality, larger file)
        """
        from matplotlib import animation
        
        print(f"Saving animation to {filename}...")
        
        if filename.endswith('.gif'):
            writer = 'pillow'
            # GIF-specific settings
            anim.save(filename, writer=writer, fps=fps, dpi=min(dpi, 80))
        else:
            # Video format (mp4, etc.)
            anim.save(filename, writer=writer, fps=fps, dpi=dpi, bitrate=bitrate)
        
        print(f"✓ Saved successfully!")


# Convenience function for quick animations
def create_quick_animation(
    flights: List[FlightPlan],
    conflicts: Optional[List[Conflict]] = None,
    view: str = '3d',
    duration: float = 10.0,
    filename: Optional[str] = None
) -> animation.FuncAnimation:
    """
    Quick helper function to create animations with default settings.
    
    Args:
        flights: Flight plans to animate
        conflicts: Optional conflicts
        view: '3d' or '2d' for perspective
        duration: Animation duration in seconds
        filename: If provided, saves to this file
        
    Returns:
        Animation object
    """
    from matplotlib import animation
    
    visualizer = FlightVisualizer()
    
    if view == '3d':
        anim = visualizer.animate_3d(
            flights, conflicts=conflicts,
            duration=duration, fps=30
        )
    else:  # 2d
        anim = visualizer.animate_2d_topdown(
            flights, conflicts=conflicts,
            duration=duration, fps=30
        )
    
    if filename:
        visualizer.save_animation(anim, filename, fps=30, dpi=100)
    
    return anim
    
