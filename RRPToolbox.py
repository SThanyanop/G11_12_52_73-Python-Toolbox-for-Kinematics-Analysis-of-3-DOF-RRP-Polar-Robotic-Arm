import math

PI = math.pi
DEG_TO_RAD = PI / 180.0
RAD_TO_DEG = 180.0 / PI
EPSILON = 1e-10


class RRPToolbox:
    def __init__(self, link_params, joint_limits):
        self.link_params = link_params
        self.joint_limits = joint_limits
        
        self.joint_local_positions = [(0, 0, 0)]
        self.joint_global_positions = [(0, 0, 0)]
        self.end_effector_position = (0, 0, 0)
        
        self.current_joint_config = [0.0, 0.0, 0.0]
        
        self._compute_link_geometry()
        
        print("RRP Toolbox Initialized (Parameters order: x, y, z)")
        print("-" * 50)
        print(f"Link Parameters       : {self.link_params}")
        print(f"Joint Limits          : {self.joint_limits}")
        print(f"Joint Local Positions : {self.joint_local_positions}")
        print(f"Joint Global Positions: {self.joint_global_positions}")
        print(f"End Effector Position : {self.end_effector_position}")
        print(f"Current Configuration : {self.current_joint_config}")
    
    def _compute_link_geometry(self):
        for sub_vec in self.link_params:
            x = sum(pos[0] for pos in sub_vec)
            y = sum(pos[1] for pos in sub_vec)
            z = sum(pos[2] for pos in sub_vec)
                
            self.joint_local_positions.append((x, y, z))
            
            prev = self.joint_global_positions[-1]
            self.joint_global_positions.append((x + prev[0], y + prev[1], z + prev[2]))
        
        self.end_effector_position = self.joint_global_positions[-1]
        
    def update_current_config(self, joint_parameters):

        self._validate_joint_parameters(joint_parameters)
        self.current_joint_config = list(joint_parameters)
        
    def get_current_config(self):

        return tuple(self.current_joint_config)
    
    def get_current_pose(self):

        return self.forward_kinematics(self.current_joint_config, update_state=False)
    
    def set_pose_from_joints(self, joint_parameters):

        position = self.forward_kinematics(joint_parameters, update_state=True)
        print(f"Pose set via joints: {joint_parameters}")
        print(f"End-effector at: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})")
        return position
    
    def set_pose_from_cartesian(self, target_position, validate=True):
        joint_params = self.inverse_kinematics(target_position, validate=validate)
        self.update_current_config(joint_params)
        print(f"Pose set via Cartesian: {target_position}")
        print(f"Joint configuration: [{joint_params[0]:.3f}, {joint_params[1]:.3f}, {joint_params[2]:.3f}]")
        return joint_params
    
    @staticmethod
    def deg_to_rad(degrees):
        return degrees * DEG_TO_RAD
    
    @staticmethod
    def rad_to_deg(radians):
        return radians * RAD_TO_DEG
    
    @staticmethod
    def matrix_multiply(A, B):

        rows_A, cols_A = len(A), len(A[0])
        cols_B = len(B[0])
        
        result = [[0.0] * cols_B for _ in range(rows_A)]
        
        for i in range(rows_A):
            A_row = A[i]
            result_row = result[i]
            for k in range(cols_A):
                A_ik = A_row[k]
                if abs(A_ik) > EPSILON:
                    B_row = B[k]
                    for j in range(cols_B):
                        result_row[j] += A_ik * B_row[j]
        
        return result
    
    @staticmethod
    def inverse_matrix(matrix):

        size = len(matrix)
        
        augmented = [matrix[i][:] + [float(i == j) for j in range(size)] for i in range(size)]
        
        for i in range(size):
            max_row = i
            max_val = abs(augmented[i][i])
            for j in range(i + 1, size):
                if abs(augmented[j][i]) > max_val:
                    max_val = abs(augmented[j][i])
                    max_row = j
            
            if max_row != i:
                augmented[i], augmented[max_row] = augmented[max_row], augmented[i]
            
            pivot = augmented[i][i]
            if abs(pivot) < EPSILON:
                raise ValueError("Matrix is singular and cannot be inverted")
            
            inv_pivot = 1.0 / pivot
            row_i = augmented[i]
            for k in range(2 * size):
                row_i[k] *= inv_pivot
            
            for j in range(size):
                if j != i:
                    factor = augmented[j][i]
                    if abs(factor) > EPSILON:
                        row_j = augmented[j]
                        for k in range(2 * size):
                            row_j[k] -= factor * row_i[k]
        
        return [row[size:] for row in augmented]
    
    @staticmethod
    def compute_determinant(matrix):

        if len(matrix) != 3 or len(matrix[0]) != 3:
            raise ValueError("Only 3x3 matrices supported")
        
        return (matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
                matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
                matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]))
    
    def dh_matrix_transform(self, a, alpha, d, theta):

        theta_rad = theta * DEG_TO_RAD
        alpha_rad = alpha * DEG_TO_RAD
 
        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)
        cos_alpha = math.cos(alpha_rad)
        sin_alpha = math.sin(alpha_rad)
        
        # Clean up near-zero values
        if abs(cos_theta) < EPSILON: cos_theta = 0.0
        if abs(sin_theta) < EPSILON: sin_theta = 0.0
        if abs(cos_alpha) < EPSILON: cos_alpha = 0.0
        if abs(sin_alpha) < EPSILON: sin_alpha = 0.0
        
        return [
            [cos_theta, -sin_theta, 0.0, a],
            [sin_theta * cos_alpha, cos_theta * cos_alpha, -sin_alpha, -sin_alpha * d],
            [sin_theta * sin_alpha, cos_theta * sin_alpha, cos_alpha, cos_alpha * d],
            [0.0, 0.0, 0.0, 1.0]
        ]
    
    def _validate_joint_parameters(self, joint_parameters):

        if len(joint_parameters) != 3:
            raise ValueError("Joint parameters must contain exactly 3 values: [theta1, theta2, d3]")
        
        theta1, theta2, d3 = joint_parameters
        
        if not (self.joint_limits[0][0] <= theta1 <= self.joint_limits[0][1]):
            raise ValueError(f"Theta1 ({theta1}) must be within limits: {self.joint_limits[0]}")
        
        if not (self.joint_limits[1][0] <= theta2 <= self.joint_limits[1][1]):
            raise ValueError(f"Theta2 ({theta2}) must be within limits: {self.joint_limits[1]}")
        
        if not (self.joint_limits[2][0] <= d3 <= self.joint_limits[2][1]):
            raise ValueError(f"d3 ({d3}) must be within limits: {self.joint_limits[2]}")
    
    def get_rrp_transform_matrix(self, joint_parameters):

        theta1, theta2, d3 = joint_parameters
        
        x1, y1, z1 = self.joint_local_positions[1]
        x2, y2, z2 = self.joint_local_positions[2]
        x3, y3, z3 = self.joint_local_positions[3]
        
        matrices = [
            self.dh_matrix_transform(0, 0, 0, theta1),
            self.dh_matrix_transform(x1, 0, z1, 0),
            self.dh_matrix_transform(0, 90, y1, theta2),
            self.dh_matrix_transform(x2, 0, y2, 90),
            self.dh_matrix_transform(z2, 90, 0, 0),
            self.dh_matrix_transform(z3, 0, x3 + d3, 90),
            self.dh_matrix_transform(y3, 90, 0, 90)
        ]
        
        T = matrices[0]
        for matrix in matrices[1:]:
            T = self.matrix_multiply(T, matrix)
            
        return T
    
    def get_rrp_jacobian_matrix(self, joint_parameters):

        theta1, theta2, d3 = joint_parameters
        
        theta1_rad = theta1 * DEG_TO_RAD
        theta2_rad = theta2 * DEG_TO_RAD
        
        c1 = math.cos(theta1_rad)
        s1 = math.sin(theta1_rad)
        c2 = math.cos(theta2_rad)
        s2 = math.sin(theta2_rad)
        
        x1, y1, z1 = self.joint_local_positions[1]
        x2, y2, z2 = self.joint_local_positions[2]
        x3, y3, z3 = self.joint_local_positions[3]
        
        y_sum = y1 + y2 + y3
        z_sum = z2 + z3
        d3_x_sum = d3 + x3 + x2
        
        s1_c2 = s1 * c2
        s1_s2 = s1 * s2
        c1_s2 = c1 * s2
        c1_c2 = c1 * c2
        s1_c2_d3 = s1_c2 * d3_x_sum
        c1_s2_d3 = c1_s2 * d3_x_sum
        c1_c2_z = c1_c2 * z_sum
        s1_s2_d3 = s1_s2 * d3_x_sum
        s1_c2_z = s1_c2 * z_sum
        
        J = [
            [-s1*x1 + c1*y_sum - s1_c2_d3 + s1_s2*z_sum,
             -c1_s2_d3 - c1_c2_z,
             c1_c2],
            [c1*x1 + s1*y_sum + c1_c2*d3_x_sum - c1_s2*z_sum,
             -s1_s2_d3 - s1_c2_z,
             s1_c2],
            [0.0,
             c2*d3_x_sum - s2*z_sum,
             s2],
            [0.0, 0.0, 0.0],  
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0]
        ]
        
        reduced_J = [J[0][:], J[1][:], J[2][:]]
        
        try:
            inv_reduced_J = self.inverse_matrix(reduced_J)
        except ValueError as e:
            raise ValueError(f"Singular configuration detected: {e}")
        
        return J, reduced_J, inv_reduced_J
    
    def check_singularity(self, joint_parameters, threshold=1e-3):
        try:
            _, reduced_J, _ = self.get_rrp_jacobian_matrix(joint_parameters)
            det = self.compute_determinant(reduced_J)
            return abs(det) < threshold
        except ValueError:
            return True
    
    def is_reachable(self, target_position):
        x, y, z = target_position

        distance = math.sqrt(x**2 + y**2 + z**2)

        x1, y1, z1 = self.joint_global_positions[1]
        x2, y2, z2 = self.joint_global_positions[2]
        x3, y3, z3 = self.joint_global_positions[3]

        d3_max = self.joint_limits[2][1]
        max_reach = math.sqrt((x1 + x2 + x3 + d3_max)**2 + (y1 + y2 + y3)**2 + (z1 + z2 + z3)**2)

        d3_min = self.joint_limits[2][0]
        min_reach = abs(d3_min) 
        
        if distance > max_reach:
            return False, f"Target too far (distance: {distance:.3f}, max: {max_reach:.3f})"
        
        if distance < min_reach:
            return False, f"Target too close (distance: {distance:.3f}, min: {min_reach:.3f})"
        
        return True, "Reachable"
    
    def forward_kinematics(self, joint_parameters, update_state=True):

        self._validate_joint_parameters(joint_parameters)
        
        T = self.get_rrp_transform_matrix(joint_parameters)
        
        if update_state:
            self.update_current_config(joint_parameters)
        
        return (T[0][3], T[1][3], T[2][3])
    
    def inverse_kinematics(self, target_position, validate=True):

        if validate:
            is_reachable, reason = self.is_reachable(target_position)
            if not is_reachable:
                raise ValueError(f"Target unreachable: {reason}")
        
        x, y, z = target_position

        x1, y1, z1 = self.joint_global_positions[1]
        x2, y2, z2 = self.joint_global_positions[2]
        x3, y3, z3 = self.joint_global_positions[3]

        theta1_offset = math.atan2(y3, x3)
        theta1 = math.atan2(y, x) - theta1_offset

        cos_neg_theta1 = math.cos(-theta1)
        sin_neg_theta1 = math.sin(-theta1)
        
        x_org = x * cos_neg_theta1 - y * sin_neg_theta1 - x1
        y_org = x * sin_neg_theta1 + y * cos_neg_theta1 - y1
        z_org = z - z1

        r_target_sq = x_org**2 + y_org**2 + z_org**2
        
        r_target = math.sqrt(r_target_sq)

        dx = x3 - x1
        dy = y3 - y1
        dz = z3 - z1
        
        dy_sq = dy * dy
        dz_sq = dz * dz
        
        discriminant = r_target_sq - dy_sq - dz_sq
        if discriminant < 0:
            raise ValueError(f"Target unreachable: invalid geometry (discriminant: {discriminant:.3f})")
        
        sqrt_term = math.sqrt(discriminant)
        
        d_minus = -dx - sqrt_term
        d_plus = -dx + sqrt_term
        
        r_ref_sq = dx**2 + dy_sq + dz_sq
        d3 = d_plus if r_target_sq > r_ref_sq else d_minus
        
        dx += d3
        
        theta2_offset = math.atan2(dz, math.sqrt(dx**2 + dy_sq))
        theta2 = math.atan2(z_org, math.sqrt(x_org**2 + y_org**2)) - theta2_offset
        
        result = (theta1 * RAD_TO_DEG, theta2 * RAD_TO_DEG, d3)
        
        try:
            self._validate_joint_parameters(result)
        except ValueError as e:
            raise ValueError(f"IK solution outside joint limits: {e}")
        
        return result
        
    def differential_forward_kinematics(self, joint_velocities, joint_config=None):
        if joint_config is None:
            joint_config = self.current_joint_config
        
        _, reduced_J, _ = self.get_rrp_jacobian_matrix(joint_config)
        
        theta1_dot = joint_velocities[0] * DEG_TO_RAD
        theta2_dot = joint_velocities[1] * DEG_TO_RAD
        d3_dot = joint_velocities[2]

        vx = reduced_J[0][0] * theta1_dot + reduced_J[0][1] * theta2_dot + reduced_J[0][2] * d3_dot
        vy = reduced_J[1][0] * theta1_dot + reduced_J[1][1] * theta2_dot + reduced_J[1][2] * d3_dot
        vz = reduced_J[2][0] * theta1_dot + reduced_J[2][1] * theta2_dot + reduced_J[2][2] * d3_dot
        
        return (vx, vy, vz)
    
    def differential_inverse_kinematics(self, target_velocities, joint_config=None):

        if joint_config is None:
            joint_config = self.current_joint_config
        
        _, _, inv_reduced_J = self.get_rrp_jacobian_matrix(joint_config)
        
        vx = target_velocities[0]
        vy = target_velocities[1]
        vz = target_velocities[2]
        
        theta1_dot = inv_reduced_J[0][0] * vx + inv_reduced_J[0][1] * vy + inv_reduced_J[0][2] * vz
        theta2_dot = inv_reduced_J[1][0] * vx + inv_reduced_J[1][1] * vy + inv_reduced_J[1][2] * vz
        d3_dot = inv_reduced_J[2][0] * vx + inv_reduced_J[2][1] * vy + inv_reduced_J[2][2] * vz
        
        return (theta1_dot * RAD_TO_DEG, theta2_dot * RAD_TO_DEG, d3_dot)
    
    def print_status(self):
        """Print current robot status including configuration, position, and singularity status"""
        print("\n" + "=" * 50)
        print("RRP Robot Status")
        print("=" * 50)
        print(f"Current Configuration: [{self.current_joint_config[0]:.3f}, "
              f"{self.current_joint_config[1]:.3f}, {self.current_joint_config[2]:.3f}]")
        
        try:
            pos = self.forward_kinematics(self.current_joint_config, update_state=False)
            print(f"End-Effector Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        except Exception as e:
            print(f"End-Effector Position: Error - {e}")
        
        is_singular = self.check_singularity(self.current_joint_config)
        print(f"Singularity Status   : {'SINGULAR' if is_singular else 'OK'}")
        print("=" * 50)
        
    def Configuration_Space_Calculation(self, target_position, target_velocity, config=False):
        
        joint_config   = self.inverse_kinematics(target_position,config)
        
        joint_velocity = self. differential_inverse_kinematics(target_velocity,config)
        
        return (joint_config,joint_velocity)
    
    def Task_Space_Calculation(self, joint_config, joint_velocity, config=False):
        
        ee_position = self.forward_kinematics(joint_config, config)
        
        ee_velocity = self.differential_forward_kinematics(joint_velocity, config)
        
        return (ee_position,ee_velocity)