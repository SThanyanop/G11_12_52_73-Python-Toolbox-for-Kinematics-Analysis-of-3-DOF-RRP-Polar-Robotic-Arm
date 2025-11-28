import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import Slider, Button

class RRPVisualization:
    
    def __init__(self, toolbox):
        self.toolbox = toolbox
    
    def get_workspace(self, theta1_samples=12, theta2_samples=12, d3_samples=6):
        workspace_points = []
        
        theta1_min, theta1_max = self.toolbox.joint_limits[0]
        theta2_min, theta2_max = self.toolbox.joint_limits[1]
        d3_min, d3_max = self.toolbox.joint_limits[2]
        
        theta1_range = [theta1_min + (theta1_max - theta1_min) * i / (theta1_samples - 1) for i in range(theta1_samples)]
        theta2_range = [theta2_min + (theta2_max - theta2_min) * i / (theta2_samples - 1) for i in range(theta2_samples)]
        d3_range = [d3_min, d3_max]  # Only sample boundary values
        
        for theta1 in theta1_range:
            for theta2 in theta2_range:
                for d3 in d3_range:
                    
                    th1_rad = self.toolbox.deg_to_rad(theta1)
                    th2_rad = self.toolbox.deg_to_rad(theta2)
                    
                    c1 = math.cos(th1_rad)
                    s1 = math.sin(th1_rad)
                    
                    current_x, current_y, current_z = 0, 0, 0
                    
                    for segment in self.toolbox.link_params[0]:
                        segment_x, segment_y, segment_z = segment
                        # Rotate by theta1 around Z axis
                        rotated_x = c1 * segment_x - s1 * segment_y
                        rotated_y = s1 * segment_x + c1 * segment_y
                        rotated_z = segment_z
                        
                        current_x += rotated_x
                        current_y += rotated_y
                        current_z += rotated_z
                    
                    c2 = math.cos(th2_rad)
                    s2 = math.sin(th2_rad)
                    
                    direction_x = s2 * c1
                    direction_y = s2 * s1
                    direction_z = c2
                    
                    for segment in self.toolbox.link_params[1]:
                        segment_length = math.sqrt(segment[0]**2 + segment[1]**2 + segment[2]**2)
                        current_x += segment_length * direction_x
                        current_y += segment_length * direction_y
                        current_z += segment_length * direction_z
                    
                    current_x += d3 * direction_x
                    current_y += d3 * direction_y
                    current_z += d3 * direction_z
                    
                    for segment in self.toolbox.link_params[2]:
                        segment_length = math.sqrt(segment[0]**2 + segment[1]**2 + segment[2]**2)
                        current_x += segment_length * direction_x
                        current_y += segment_length * direction_y
                        current_z += segment_length * direction_z
                    
                    workspace_points.append((current_x, current_y, current_z))
        
        return workspace_points
    
    def find_singularities(self, theta1_samples=20, theta2_samples=20, d3_samples=10):
        singularity_positions = []
        singularity_configs = []
        singularity_threshold = 1e-3
        
        theta1_min, theta1_max = self.toolbox.joint_limits[0]
        theta2_min, theta2_max = self.toolbox.joint_limits[1]
        d3_min, d3_max = self.toolbox.joint_limits[2]
        
        def linspace(start, end, samples):
            if samples == 1:
                return [start]
            return [start + (end - start) * i / (samples - 1) for i in range(samples)]

        theta1_range = linspace(theta1_min, theta1_max, theta1_samples)
        theta2_range = linspace(theta2_min, theta2_max, theta2_samples)
        d3_range     = linspace(d3_min,     d3_max,     d3_samples)
        
        for theta1 in theta1_range:
            for theta2 in theta2_range:
                for d3 in d3_range:
                    try:
                        is_singular = self.toolbox.check_singularity([theta1, theta2, d3], threshold=singularity_threshold)
                        if is_singular:
                            position = self.toolbox.forward_kinematics([theta1, theta2, d3], update_state=False)
                            singularity_positions.append(position)
                            singularity_configs.append(([theta1, theta2, d3], position))
                    except:
                        pass
        
        return singularity_positions, singularity_configs
    
    def _compute_visual_positions(self, theta1, theta2, d3):
        theta1_rad = np.radians(theta1)
        theta2_rad = np.radians(theta2)
        
        def Rz(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        
        positions = [np.array([0, 0, 0])]
        
        current_pos = np.array([0, 0, 0])
        for segment in self.toolbox.link_params[0]:
            segment_vec = np.array(segment)
            rotated_segment = Rz(theta1_rad) @ segment_vec
            current_pos = current_pos + rotated_segment
            positions.append(current_pos.copy())
        
        direction = np.array([
            np.sin(theta2_rad) * np.cos(theta1_rad),
            np.sin(theta2_rad) * np.sin(theta1_rad),
            np.cos(theta2_rad)
        ])
        
        for segment in self.toolbox.link_params[1]:
            segment_length = np.linalg.norm(segment)
            current_pos = current_pos + segment_length * direction
            positions.append(current_pos.copy())
        
        current_pos = current_pos + d3 * direction
        positions.append(current_pos.copy())
        
        for segment in self.toolbox.link_params[2]:
            segment_length = np.linalg.norm(segment)
            current_pos = current_pos + segment_length * direction
            positions.append(current_pos.copy())
        
        return np.array(positions)
    
    def plot_robot(self, theta1=0, theta2=0, d3=0, ax=None, show_frame=True):
        if ax is None:
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
        
        positions = self._compute_visual_positions(theta1, theta2, d3)
        
        link1_end_idx = len(self.toolbox.link_params[0])
        link2_end_idx = link1_end_idx + len(self.toolbox.link_params[1])
        d3_end_idx = link2_end_idx + 1
        
        link1_positions = positions[:link1_end_idx+1]
        ax.plot(link1_positions[:, 0], link1_positions[:, 1], link1_positions[:, 2], 
                'o-', linewidth=3, markersize=8, color='steelblue', label='Link 1')
        
        link2_positions = positions[link1_end_idx:link2_end_idx+1]
        ax.plot(link2_positions[:, 0], link2_positions[:, 1], link2_positions[:, 2], 
                'o-', linewidth=3, markersize=8, color='coral', label='Link 2')
        
        d3_positions = positions[link2_end_idx:d3_end_idx+1]
        ax.plot(d3_positions[:, 0], d3_positions[:, 1], d3_positions[:, 2], 
                'o-', linewidth=4, markersize=8, color='green', label='Prismatic (d3)')
        
        if len(self.toolbox.link_params[2]) > 0:
            ee_positions = positions[d3_end_idx:]
            ax.plot(ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2], 
                    's-', linewidth=2, markersize=10, color='red', label='End Effector')
        
        ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                  c='red', s=150, marker='o', label='Base', zorder=5)
        
        x1, y1, z1 = self.toolbox.joint_global_positions[1]
        x2, y2, z2 = self.toolbox.joint_global_positions[2]
        x3, y3, z3 = self.toolbox.joint_global_positions[3]
        d3_max = self.toolbox.joint_limits[2][1]
        
        max_reach = math.sqrt((x1 + x2 + x3 + d3_max)**2 + (y1 + y2 + y3)**2 + (z1 + z2 + z3)**2) + 0.5
        
        ax.set_xlim([-max_reach, max_reach])
        ax.set_ylim([-max_reach, max_reach])
        ax.set_zlim([0, max_reach*1.5])
        
        ax.view_init(elev=20, azim=45)
        
        ax.set_xlabel('X (m)', fontsize=10)
        ax.set_ylabel('Y (m)', fontsize=10)
        ax.set_zlabel('Z (m)', fontsize=10)
        ax.set_title(f'RRP Robot Configuration\nθ1={theta1:.1f}°, θ2={theta2:.1f}°, d3={d3:.2f}m', 
                    fontsize=12)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        return ax
    
    def plot_workspace_3d(self, theta1_samples=10, theta2_samples=10, d3_samples=5):
        workspace_points = self.get_workspace(theta1_samples, theta2_samples, d3_samples)
        
        if not workspace_points:
            print("No valid workspace points generated.")
            return

        print("Finding singularities...")
        singularity_positions, singularity_configs = self.find_singularities(
            theta1_samples=max(8, theta1_samples//2), 
            theta2_samples=max(8, theta2_samples//2), 
            d3_samples=max(5, d3_samples//2)
        )
        print(f"Found {len(singularity_positions)} singularity positions")

        all_points = workspace_points + singularity_positions
        
        if len(all_points) < 4:
            print("Not enough points to compute convex hull (need at least 4).")
            return

        points = np.array(all_points)
        ws_array = np.array(workspace_points)

        fig = plt.figure(figsize=(16, 11))
        ax = fig.add_subplot(111, projection='3d')

        ax_elev = fig.add_axes([0.2, 0.05, 0.3, 0.03])
        ax_azim = fig.add_axes([0.2, 0.01, 0.3, 0.03])
        ax_toggle_vertices = fig.add_axes([0.65, 0.05, 0.15, 0.03])
        ax_toggle_faces = fig.add_axes([0.65, 0.01, 0.15, 0.03])

        slider_elev = Slider(ax_elev, 'Elevation', -90, 90, valinit=0, color='orange')
        slider_azim = Slider(ax_azim, 'Azimuth', -180, 180, valinit=0, color='cyan')

        btn_toggle_vertices = Button(ax_toggle_vertices, 'Hide Vertices', color='lightgray', hovercolor='0.975')
        btn_toggle_faces = Button(ax_toggle_faces, 'Hide Faces', color='lightgray', hovercolor='0.975')

        toggle_state = {'vertices_visible': True, 'faces_visible': True}
        vertex_scatter = None
        face_collections = []
        
        def update_view(val):
            
            ax.view_init(elev=slider_elev.val, azim=slider_azim.val)
            fig.canvas.draw_idle()
        
        def toggle_vertices(event):
            
            toggle_state['vertices_visible'] = not toggle_state['vertices_visible']
            if vertex_scatter is not None:
                vertex_scatter.set_visible(toggle_state['vertices_visible'])
            btn_toggle_vertices.label.set_text('Hide Vertices' if toggle_state['vertices_visible'] else 'Show Vertices')
            fig.canvas.draw_idle()
        
        def toggle_faces(event):
            
            toggle_state['faces_visible'] = not toggle_state['faces_visible']
            for face_coll in face_collections:
                face_coll.set_visible(toggle_state['faces_visible'])
            btn_toggle_faces.label.set_text('Hide Faces' if toggle_state['faces_visible'] else 'Show Faces')
            fig.canvas.draw_idle()
        
        slider_elev.on_changed(update_view)
        slider_azim.on_changed(update_view)
        btn_toggle_vertices.on_clicked(toggle_vertices)
        btn_toggle_faces.on_clicked(toggle_faces)

        slider_elev.set_val(0)
        slider_azim.set_val(0)

        theta1_min, theta1_max = self.toolbox.joint_limits[0]
        theta2_min, theta2_max = self.toolbox.joint_limits[1]
        d3_min, d3_max = self.toolbox.joint_limits[2]
        
        theta1_range = [theta1_min + (theta1_max - theta1_min) * i / (theta1_samples - 1) for i in range(theta1_samples)]
        theta2_range = [theta2_min + (theta2_max - theta2_min) * i / (theta2_samples - 1) for i in range(theta2_samples)]

        point_idx = 0
        point_map = {}
        for i in range(theta1_samples):
            for j in range(len(theta2_range)):
                for d3_idx in range(2):  
                    point_map[(i, j, d3_idx)] = point_idx
                    point_idx += 1

        for i in range(theta1_samples):
            for j in range(len(theta2_range)):
                is_theta2_boundary = (j == 0 or j == len(theta2_range) - 1)
                
                idx_min = point_map.get((i, j, 0))
                idx_max = point_map.get((i, j, 1))
                
                if idx_min is not None and idx_max is not None and idx_min < len(ws_array) and idx_max < len(ws_array):
                    if is_theta2_boundary:
                        p_min = ws_array[idx_min]
                        p_max = ws_array[idx_max]
                        ax.plot3D([p_min[0], p_max[0]], [p_min[1], p_max[1]], [p_min[2], p_max[2]], 
                                 'b-', linewidth=1, alpha=0.5)

        for i in [0, theta1_samples - 1]:
            for j in range(len(theta2_range)):
                idx_min = point_map.get((i, j, 0))
                idx_max = point_map.get((i, j, 1))
                
                if idx_min is not None and idx_max is not None and idx_min < len(ws_array) and idx_max < len(ws_array):
                    p_min = ws_array[idx_min]
                    p_max = ws_array[idx_max]
                    ax.plot3D([p_min[0], p_max[0]], [p_min[1], p_max[1]], [p_min[2], p_max[2]], 
                             'b-', linewidth=1, alpha=0.5)

        for i in range(theta1_samples):
            for d3_idx in range(2):
                for j in range(len(theta2_range) - 1):
                    idx_curr = point_map.get((i, j, d3_idx))
                    idx_next = point_map.get((i, j + 1, d3_idx))
                    
                    if idx_curr is not None and idx_next is not None and idx_curr < len(ws_array) and idx_next < len(ws_array):
                        p_curr = ws_array[idx_curr]
                        p_next = ws_array[idx_next]
                        ax.plot3D([p_curr[0], p_next[0]], [p_curr[1], p_next[1]], [p_curr[2], p_next[2]], 
                                 'b-', linewidth=1, alpha=0.5)

        for j in range(len(theta2_range)):
            for d3_idx in range(2):
                for i in range(theta1_samples - 1):
                    idx_curr = point_map.get((i, j, d3_idx))
                    idx_next = point_map.get((i + 1, j, d3_idx))
                    
                    if idx_curr is not None and idx_next is not None and idx_curr < len(ws_array) and idx_next < len(ws_array):
                        p_curr = ws_array[idx_curr]
                        p_next = ws_array[idx_next]
                        ax.plot3D([p_curr[0], p_next[0]], [p_curr[1], p_next[1]], [p_curr[2], p_next[2]], 
                                 'b-', linewidth=1, alpha=0.5)

        faces = []
        face_set = set()

        for i in range(theta1_samples - 1):
            for d3_idx in range(2):
                for j in range(len(theta2_range) - 1):
                    idx1 = point_map.get((i, j, d3_idx))
                    idx2 = point_map.get((i + 1, j, d3_idx))
                    idx3 = point_map.get((i + 1, j + 1, d3_idx))
                    idx4 = point_map.get((i, j + 1, d3_idx))
                    
                    if all(idx is not None and idx < len(ws_array) for idx in [idx1, idx2, idx3, idx4]):
                        face_key = tuple(sorted([idx1, idx2, idx3, idx4]))
                        if face_key not in face_set:
                            face_set.add(face_key)
                            p1 = ws_array[idx1]
                            p2 = ws_array[idx2]
                            p3 = ws_array[idx3]
                            p4 = ws_array[idx4]
                            faces.append([p1, p2, p3, p4])

        for j in [0, len(theta2_range) - 1]:
            for i in range(theta1_samples - 1):
                idx1 = point_map.get((i, j, 0))
                idx2 = point_map.get((i + 1, j, 0))
                idx3 = point_map.get((i + 1, j, 1))
                idx4 = point_map.get((i, j, 1))
                
                if all(idx is not None and idx < len(ws_array) for idx in [idx1, idx2, idx3, idx4]):
                    face_key = tuple(sorted([idx1, idx2, idx3, idx4]))
                    if face_key not in face_set:
                        face_set.add(face_key)
                        p1 = ws_array[idx1]
                        p2 = ws_array[idx2]
                        p3 = ws_array[idx3]
                        p4 = ws_array[idx4]
                        faces.append([p1, p2, p3, p4])

        for i in [0, theta1_samples - 1]:
            for j in range(len(theta2_range) - 1):
                idx1 = point_map.get((i, j, 0))
                idx2 = point_map.get((i, j + 1, 0))
                idx3 = point_map.get((i, j + 1, 1))
                idx4 = point_map.get((i, j, 1))
                
                if all(idx is not None and idx < len(ws_array) for idx in [idx1, idx2, idx3, idx4]):
                    face_key = tuple(sorted([idx1, idx2, idx3, idx4]))
                    if face_key not in face_set:
                        face_set.add(face_key)
                        p1 = ws_array[idx1]
                        p2 = ws_array[idx2]
                        p3 = ws_array[idx3]
                        p4 = ws_array[idx4]
                        faces.append([p1, p2, p3, p4])

        if faces:
            face_collection = Poly3DCollection(faces, alpha=0.25, facecolor='cyan', edgecolor='none')
            face_collection.set_visible(toggle_state['faces_visible'])
            ax.add_collection3d(face_collection)
            face_collections.append(face_collection)

        z_values_ws = ws_array[:, 2]
        vertex_scatter = ax.scatter(ws_array[:, 0], 
                            ws_array[:, 1], 
                            ws_array[:, 2],
                            c=z_values_ws, cmap='viridis', marker='o', s=40, alpha=0.8, 
                            label='Workspace Grid Vertices', edgecolors='black', linewidth=0.5)

        cbar = plt.colorbar(vertex_scatter, ax=ax, pad=0.1, shrink=0.8)
        cbar.set_label('Height (Z-axis)', rotation=270, labelpad=15)

        if singularity_positions:
            sing_array = np.array(singularity_positions)
            ax.scatter(sing_array[:, 0], 
                      sing_array[:, 1], 
                      sing_array[:, 2],
                      c='purple', marker='X', s=200, alpha=0.9, 
                      label=f'Singularities ({len(singularity_positions)})', 
                      edgecolors='black', linewidth=1.5)

        joint_params = [self.toolbox.joint_limits[0][0], self.toolbox.joint_limits[1][0], self.toolbox.joint_limits[2][1]]
        
        theta1 = joint_params[0]
        theta2 = joint_params[1]
        d3 = joint_params[2]
        
        th1 = np.radians(theta1)
        th2 = np.radians(theta2)

        robot_positions = [np.array([0, 0, 0])]
        current_pos = np.array([0, 0, 0])
        
        def Rz(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        
        direction = np.array([
            np.sin(th2) * np.cos(th1),
            np.sin(th2) * np.sin(th1),
            np.cos(th2)
        ])

        for segment in self.toolbox.link_params[0]:
            segment_vec = np.array(segment)
            rotated_segment = Rz(th1) @ segment_vec
            current_pos = current_pos + rotated_segment
            robot_positions.append(current_pos.copy())
        
        link1_end_idx = len(robot_positions) - 1

        for segment in self.toolbox.link_params[1]:
            segment_length = np.linalg.norm(np.array(segment))
            current_pos = current_pos + segment_length * direction
            robot_positions.append(current_pos.copy())
        
        link2_end_idx = len(robot_positions) - 1

        current_pos = current_pos + d3 * direction
        robot_positions.append(current_pos.copy())
        
        d3_end_idx = len(robot_positions) - 1

        for segment in self.toolbox.link_params[2]:
            segment_length = np.linalg.norm(np.array(segment))
            current_pos = current_pos + segment_length * direction
            robot_positions.append(current_pos.copy())
        
        robot_array = np.array(robot_positions)

        link1_positions = robot_array[:link1_end_idx + 1]
        ax.plot3D(link1_positions[:, 0], link1_positions[:, 1], link1_positions[:, 2], 
                 'o-', linewidth=3, markersize=8, color='steelblue', label='Link 1')

        link2_positions = robot_array[link1_end_idx:link2_end_idx + 1]
        ax.plot3D(link2_positions[:, 0], link2_positions[:, 1], link2_positions[:, 2], 
                 'o-', linewidth=3, markersize=8, color='coral', label='Link 2')

        d3_positions = robot_array[link2_end_idx:d3_end_idx + 1]
        ax.plot3D(d3_positions[:, 0], d3_positions[:, 1], d3_positions[:, 2], 
                 'o-', linewidth=4, markersize=8, color='green', label='Prismatic (d3)')

        if len(self.toolbox.link_params[2]) > 0:
            ee_positions = robot_array[d3_end_idx:]
            ax.plot3D(ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2], 
                     's-', linewidth=2, markersize=10, color='red', label='End Effector')

        ax.scatter(robot_array[0, 0], robot_array[0, 1], robot_array[0, 2], 
                  c='green', s=150, marker='s', label='Base', zorder=5)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        x_max = max(abs(points[:, 0].min()), abs(points[:, 0].max()))
        y_max = max(abs(points[:, 1].min()), abs(points[:, 1].max()))
        z_max = max(abs(points[:, 2].min()), abs(points[:, 2].max()))
        
        max_distance = max(x_max, y_max, z_max)
        margin = max_distance * 0.1
        axis_limit = max_distance + margin
        
        ax.set_xlim(-axis_limit, axis_limit)
        ax.set_ylim(-axis_limit, axis_limit)
        ax.set_zlim(-axis_limit, axis_limit)

        q1_min, q1_max = self.toolbox.joint_limits[0]
        q2_min, q2_max = self.toolbox.joint_limits[1]
        d3_min_val, d3_max_val = self.toolbox.joint_limits[2]
        
        title_text = f'RRP Robot Workspace with Singularities (Interactive)\n'
        title_text += f'Configuration: θ1={joint_params[0]:.1f}°, θ2={joint_params[1]:.1f}°, d3={joint_params[2]:.1f}\n'
        title_text += f'Limits - θ1: [{q1_min:.1f}°, {q1_max:.1f}°], θ2: [{q2_min:.1f}°, {q2_max:.1f}°], d3: [{d3_min_val:.1f}, {d3_max_val:.1f}]'
        
        ax.set_title(title_text, fontsize=10)
        ax.legend(loc='upper left', fontsize=9)

        ax.view_init(elev=20, azim=45)
        
        plt.tight_layout(rect=[0, 0.08, 1, 1])
        plt.show()

    def interactive_plot(self):
        
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        plt.subplots_adjust(bottom=0.25)

        theta1_init = (self.toolbox.joint_limits[0][0] + self.toolbox.joint_limits[0][1]) / 2
        theta2_init = (self.toolbox.joint_limits[1][0] + self.toolbox.joint_limits[1][1]) / 2
        d3_init = (self.toolbox.joint_limits[2][0] + self.toolbox.joint_limits[2][1]) / 2

        ax_theta1 = plt.axes([0.15, 0.15, 0.65, 0.03])
        ax_theta2 = plt.axes([0.15, 0.10, 0.65, 0.03])
        ax_d3 = plt.axes([0.15, 0.05, 0.65, 0.03])
        
        slider_theta1 = Slider(ax_theta1, 'θ1 (°)', *self.toolbox.joint_limits[0], 
                              valinit=theta1_init, valstep=1)
        slider_theta2 = Slider(ax_theta2, 'θ2 (°)', *self.toolbox.joint_limits[1], 
                              valinit=theta2_init, valstep=1)
        slider_d3 = Slider(ax_d3, 'd3 (m)', *self.toolbox.joint_limits[2], 
                          valinit=d3_init, valstep=0.01)
        
        def update(val):
            ax.cla()
            theta1 = slider_theta1.val
            theta2 = slider_theta2.val
            d3 = slider_d3.val
            self.plot_robot(theta1, theta2, d3, ax=ax)
            
            fig.canvas.draw_idle()
        
        slider_theta1.on_changed(update)
        slider_theta2.on_changed(update)
        slider_d3.on_changed(update)

        self.plot_robot(theta1_init, theta2_init, d3_init, ax=ax)
        plt.show()

    def interpolate_trajectory(self, waypoints, total_time, fps=30):

        num_waypoints = len(waypoints)
        times = np.linspace(0, total_time, num_waypoints)
        
        trajectory = []
        time_stamps = []
        
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            t_start = times[i]
            t_end = times[i + 1]
            duration = t_end - t_start
            
            if duration <= 0:
                continue

            num_frames = int(duration * fps)

            for j in range(num_frames):
                alpha = j / num_frames
                interpolated = tuple(
                    start[k] + alpha * (end[k] - start[k]) 
                    for k in range(len(start))
                )
                trajectory.append(interpolated)
                time_stamps.append(t_start + alpha * duration)

        trajectory.append(waypoints[-1])
        time_stamps.append(times[-1])
        
        return trajectory, time_stamps
    
    def animate_trajectory(self, trajectory, total_time=None, trajectory_type='joint', fps=30):

        if total_time is None:
            total_time = len(trajectory)  

        if trajectory_type == 'position':
            
            joint_waypoints = []
            for pos in trajectory:
                x, y, z = pos
                joint_config = self.toolbox.inverse_kinematics((x, y, z), validate=True)
                joint_waypoints.append(joint_config)
        
            joint_trajectory, time_stamps = self.interpolate_trajectory(joint_waypoints, total_time, fps)
        else:
            joint_trajectory, time_stamps = self.interpolate_trajectory(trajectory, total_time, fps)

        actual_ee_path = []
        for joint_config in joint_trajectory:
            theta1, theta2, d3 = joint_config
            positions = self._compute_visual_positions(theta1, theta2, d3)
            
            actual_ee_path.append(positions[-1])
        path_points = np.array(actual_ee_path)
    
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
    
        def update_frame(frame):
            ax.cla()
            theta1, theta2, d3 = joint_trajectory[frame]
            self.plot_robot(theta1, theta2, d3, ax=ax, show_frame=False)

            ax.plot(path_points[:frame+1, 0], path_points[:frame+1, 1], 
                path_points[:frame+1, 2], 'r--', linewidth=2, 
                alpha=0.5, label='End Effector Path')
            ax.scatter(path_points[frame, 0], path_points[frame, 1], 
                    path_points[frame, 2], s=100, c='yellow', marker='*', 
                    edgecolors='black', linewidths=2, zorder=10)
        
            current_time = time_stamps[frame]
            total_time_display = time_stamps[-1]
            ee_pos = path_points[frame]
        
            ax.set_title(f'Time: {current_time:.2f}s / {total_time_display:.2f}s\n' + 
                        f'θ1={theta1:.1f}°, θ2={theta2:.1f}°, d3={d3:.2f}m\n' +
                        f'End Effector: [{ee_pos[0]:.2f}, {ee_pos[1]:.2f}, {ee_pos[2]:.2f}]')
            ax.legend(loc='upper right')
    
        interval = 1000 / fps  
        anim = FuncAnimation(fig, update_frame, frames=len(joint_trajectory), 
                       interval=interval, repeat=True)
        plt.show()
        return anim