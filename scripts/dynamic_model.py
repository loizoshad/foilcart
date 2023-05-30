import numpy as np
import casadi as ca
import re

class DynamicModel:
    '''
    State vector:
    x = [x, y, z, phi, theta, psi, vx, vy, vz, wx, wy, wz]
        - x(0): x position [m]          x(1): y position [m]          x(2): z position [m]
        - x(3): phi angle [rad]         x(4): theta angle [rad]       x(5): psi angle [rad]
        - x(6): vx velocity [m/s]       x(7): vy velocity [m/s]       x(8): vz velocity [m/s]
        - x(9): wx ang. vel. [rad/s]    x(10): wy ang. vel. [rad/s]   x(11): wz ang. vel. [rad/s]

    Control vector:
    u = [f_strb, f_port, wing, rudder, elevator]
        - u(0): f_strb   - thrust force generated by the starboard (left) motor in [N]
        - u(1): f_port   - thrust force generated by the port (right) motor in [N]
        - u(2): wing     - angle of the main wing in [rad]
        - u(3): rudder   - angle of the rudder in [rad]
        - u(4): elevator - angle of the elevator in [rad]

    Coordinate system:
    - x positive semi-axis: forward,    y-axis: right,    z positive semi-axis: down
    - Rotations: Positive angles are clockwise when looking from the origin towards the positive direction of the axis. (right hand rule)    
    '''

    NS = 12
    NC = 5

    x = ca.SX.sym("x", NS);        # State vector
    u = ca.SX.sym("u", NC);        # Control vector

    def __init__(self) -> None:
        # All dimensions are taken from the main simulink model [1], (not the theses from Henrik [2] and Adam [3]).
        g = 9.81;                     # (m/s^2) Gravitational acceleration.
        rho = 997.0;                  # (kg/m^3) Density of medium (water).
        m = 150.85;                   # (kg) Total mass of the boat.
        Izz = 20.0;                   # (kgm^2) Moment of inertia for axis z.
        Iyy = 20.5;                   # (kgm^2) Moment of inertia for axis y.
        Ixx = 3.9;                    # (kgm^2) Moment of inertia for axis x.
        A_w_strb = 0.5*0.2;           # NACA0015 200x500
        A_w_port = 0.5*0.2;           # NACA0015 200x500
        A_w = A_w_strb + A_w_port
        A_e = 0.5*0.1;                # NACA0015 100x500
        A_r = 0.5*0.2;                # NACA0015 200x500
        L1 = 0.50;                    # (m  - Distance in x-axis between CoG and keel. GUESSTIMATE
        L2 = 1.25;                    # (m) - Distance in x-axis between keel and elevator.
        L3 = 0.25;                    # (m) - Distance in y-axis between CoG and a motor.
        L4 = 1.00;                    # (m) - Distance in z-axis between local origo and the fuselage.
        L5 = 0.50;                    # (m) - Distance in z-axis between CoG and the local origo. GUESSTIMATE
        L6 = 0.25;                    # (m) - Distance in x-axis between keel base and main wing lift/drag center.

        # Lift and drag coefficients (These equations are taken from the work in [2])
        C_D_r = 0.0018*self.u[3]*self.u[3] + 0.0314;                                           #  Drag coefficient for the rudder
        C_L_r = 0.0760*self.u[3];                                                         #  Lift coefficient for the rudder
        C_D_w = 0.0018*(self.u[2] + self.x[4])*(self.u[2] + self.x[4]) - 0.003*(self.u[2] + self.x[4]) + 0.0314;   #  Drag coefficient for the main wing
        C_L_w = 0.076*(self.u[2] + self.x[4]) + 0.35;                                          #  Lift coefficient for the main wing
        C_D_e = 0.0018*(self.u[4] + self.x[4])*(self.u[4] + self.x[4]) - 0.003*(self.u[4] + self.x[4]) + 0.0314;   #  Drag coefficient for the elevator
        C_L_e = 0.076*(self.u[4] + self.x[4]);                                                 #  Lift coefficient for the elevator

        # Forces and torques
        # Forces (Drag forces f3, f5, and f7 are defined positive along the negative x-axis)
        f1 = self.u[0];                         #  Thrust force generated by the starboard motor
        f2 = self.u[1];                         #  Thrust force generated by the port motor
        f3 = (0.5)*(rho*C_D_r*A_r*self.x[6]*self.x[6])    # Rudder drag      (Positive -> Positive x-axis (Forward) )
        f4 = (0.5)*(rho*C_L_r*A_r*self.x[6]*self.x[6])    # Rudder lift      (Positive -> Positive y-axis (Right)   )
        f5 = (0.5)*(rho*C_D_w*A_w*self.x[6]*self.x[6])    # Main wing drag   (Positive -> Positive x-axis (Forward) )
        f6 = (0.5)*(rho*C_L_w*A_w*self.x[6]*self.x[6])    # Main wing lift   (Positive -> Negatice z-axis (Upwards) )
        f7 = (0.5)*(rho*C_D_e*A_e*self.x[6]*self.x[6])    # Elevator drag    (Positive -> Positive x-axis (Forward) )
        f8 = (0.5)*(rho*C_L_e*A_e*self.x[6]*self.x[6])    # Elevator lift    (Positive -> Negative z-axis (Upwards) )
        # Torques
        tau_mot_starb = ca.cross(   ca.vertcat( -(L1 - L6), -L3, L4),
                                    ca.vertcat( f1 * ca.cos(self.u[2]), 0.0, -f1 * ca.sin(self.u[2])))

        tau_mot_port = ca.cross(    ca.vertcat( -(L1 - L6), L3, L4),
                                    ca.vertcat( f2 * ca.cos(self.u[2]), 0.0, -f2 * ca.sin(self.u[2])))

        tau_keel = ca.cross(        ca.vertcat( -L1, 0.0, L4),
                                    ca.vertcat( -f3, f4, 0.0))

        tau_main = ca.cross(        ca.vertcat( -(L1 - L6), 0.0, L4),
                                    ca.vertcat( -f5, 0.0, -f6))

        tau_elev = ca.cross(        ca.vertcat( -(L1 + L2), 0.0, L4),
                                    ca.vertcat( -f7, 0.0, -f8))
        
        #  #  TODO: Derive the correct expression for the gravity torque
        # tau_grav = ca.cross(        ca.vertcat([0.0, 0.0, -L5]),
        #                             ca.vertcat([m*g*(-ca.cos(self.x[3])*ca.cos(self.x[5])*ca.sin(self.x[4]) + ca.sin(self.x[3])*ca.sin(self.x[5])),
        #                                         m*g*(ca.cos(self.x[5])*ca.sin(self.x[3]) + ca.cos(self.x[3])*ca.sin(self.x[4])*ca.sin(self.x[5])),
        #                                         m*g*(ca.cos(self.x[3])*ca.cos(self.x[4]))]))
        tau_grav = ca.cross(        ca.vertcat( 0.0, 0.0, 0.0),
                                    ca.vertcat( 0.0, 0.0, 0.0))

        #  Rotation matrices

        T_X = ca.vertcat(   ca.horzcat(1.0, 0.0, 0.0),
                            ca.horzcat(0.0, ca.cos(self.x[3]), -ca.sin(self.x[3])),
                            ca.horzcat(0.0, ca.sin(self.x[3]), ca.cos(self.x[3])))
        
        T_Y = ca.vertcat(   ca.horzcat(ca.cos(self.x[4]), 0.0, ca.sin(self.x[4])),
                            ca.horzcat(0.0, 1.0, 0.0),
                            ca.horzcat(-ca.sin(self.x[4]), 0.0, ca.cos(self.x[4])))
        
        T_Z = ca.vertcat(   ca.horzcat(ca.cos(self.x[5]), -ca.sin(self.x[5]), 0.0),
                            ca.horzcat(ca.sin(self.x[5]), ca.cos(self.x[5]), 0.0),
                            ca.horzcat(0.0, 0.0, 1.0))
        
        T_rot = ca.mtimes(T_Z, ca.mtimes(T_Y, T_X))

        #  Total forces and torques
        F_act = ca.vertcat( (f1 + f2)*ca.cos(self.u[2]) - f3 - f5 - f7,
                            f4,
                            - f6 - f8 - (f1 + f2)*ca.sin(self.u[2]) )
        
        F_grav = ca.vertcat(0.0,
                            0.0,
                            m*g )

        F_tot = ca.mtimes(T_rot, F_act) + F_grav
        M_tot = tau_mot_starb + tau_mot_port + tau_keel + tau_main + tau_elev + tau_grav

        #  Nonlinear model
        self.h1 = self.x[6]          #  x(0): x position [m]
        self.h2 = self.x[7]          #  x(1): y position [m]
        self.h3 = self.x[8]          #  x(2): z position [m]
        self.h4 = self.x[9]          #  x(3): phi angle [rad]
        self.h5 = self.x[10]         #  x(4): theta angle [rad]
        self.h6 = self.x[11]         #  x(5): psi angle [rad]
        self.h7 = F_tot[0]/m         #  x(6): vx velocity [m/s]
        self.h8 = F_tot[1]/m         #  x(7): vy velocity [m/s]
        self.h9 = F_tot[2]/m         #  x(8): vz velocity [m/s]
        self.h10 = M_tot[0]/Ixx      #  x(9): wx ang. vel. [rad/s]
        self.h11 = M_tot[1]/Iyy      #  x(10): wy ang. vel. [rad/s]
        self.h12 = M_tot[2]/Izz      #  x(11): wz ang. vel. [rad/s]


        ################################################################
        # Measurement model (linearized)                               #
        ################################################################
        # TODO: Define the nonlinear measurement model




    def compute_dynamics_jacobian(self):
        # Symbollically evaluate the jacobiann for the nonlinear model
        self.h = ca.vertcat(self.h1, self.h2, self.h3, self.h4, self.h5, self.h6, self.h7, self.h8, self.h9, self.h10, self.h11, self.h12)
        self.var = ca.vertcat(self.x, self.u)
        self.J = ca.jacobian(self.h, self.var)

        # Create an empty list of strings to store the symbolic jacobian
        self.J_str = []
        A = []
        B = []

        for i in range(0, self.NS):
            for j in range(0, self.NS):
                self.J_str.append( convert_alias(str(self.J[i,j])) )
                self.J_str[-1] = add_decimal_points(self.J_str[-1])
                A.append(self.J_str[-1])

            for j in range(self.NS, self.NS + self.NC):
                self.J_str.append( convert_alias(str(self.J[i,j])) )
                self.J_str[-1] = add_decimal_points(self.J_str[-1])
                B.append(self.J_str[-1])


        return A, B
      

def convert_alias(s):
    '''
    '''

    # Split the string based on commas
    s = s.split(',')
    final = s[-1]
    
    # print(f'len(s) = {len(s)}')

    for i in range(len(s) - 1):
        alias = re.findall(r'@[^=]+', s[(len(s) - 2) - i])
        expr = re.findall(r'=([^,]+)', s[(len(s) - 2) - i])
        # print(f'alias = {alias[0]}, expr = {expr[0]}')
        final = final.replace(alias[0], expr[0])

    final = re.sub(r'x_(\d+)', r'x[\1]', final) # Replace all the x_i with x[i]
    final = re.sub(r'u_(\d+)', r'u[\1]', final) # Replace all the u_i with u[i]

    if final == '00':   # If the entire final expression is just 00, then replace it with 0.0
        final = '0.0'
    if final =='1':     # If the entire final expression is just 1, then replace it with 1.0
        final = '1.0'
            
    return final


def add_decimal_points(s):

    '''
    Given a string, find all the numbers that are following are followed by an operator (+, -, *, /, ^) and that do not already
    have a decimal point. Then add a decimal point after the number and a zero after the decimal point.

    Example:
        Input: s=u[1]+2*x[3]*113+34.4*8

        1. Scan the entire string sequentially and store each numerical expression alongside its preceding and proceeding characters.
            numbers = ['+2*', '*113+', '+34.4*', '*8'}

        2. Check if the character before or after the number is an operator
            3. If yes, then check if the number already has a decimal point.
                4. If yes, then do nothing.
                5. If no, then add a decimal point and a zero after the number.
            6. If no, then do nothing.
        7. Repeat steps 2-6 until the end of the string is reached.

        Output: u[1] + 2*x[3]*113.0 + 34.4*8.0    
    '''

    # Match any digit sequence with at most one period outside square brackets
    pattern = r'(?<!\[)\d*\.?\d+(?!\])'
    
    def replace(match):
        # Check if the match already contains a period
        if '.' in match.group():
            return match.group()
        else:
            # Add a period to the end of the match
            return match.group() + '.0'
    
    # Use re.sub to replace each match with the result of the replace function
    s = re.sub(pattern, replace, s)
    
    

    return s



