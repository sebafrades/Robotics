# Arduino Inverse Kinematics 5-DOF Robot Arm

An Arduino-based control system for a 5-DOF robotic arm utilizing Inverse Kinematics (IK) and spatial path planning. The system features a non-blocking Finite State Machine (FSM) driven via Serial communication to execute smooth 3D linear trajectories and arbitrarily oriented 3D circular paths.

Developed as a Final Project for **Elementos de Medición, Supervisión y Control de Máquinas** (Mechanical Engineering Department, Universidad Nacional del Comahue, 2022).

---

## 📌 Project Features

- **Inverse Kinematic Solver:** Translates target Cartesian $(X, Y, Z)$ coordinates into joint servo angles ($\theta_0, \theta_1, \theta_2, \theta_3$) in real-time[cite: 1].
- **3D Linear Trajectory Generation:** Interpolates spatial points along a 3D line vector connecting user-defined start and end coordinates[cite: 1].
- **3D Circular Trajectory Generation:** Uses a $Z-X-Z$ Euler rotation matrix to calculate and execute circles tilted along any plane in 3D space[cite: 1].
- **Finite State Machine (FSM):** Structured `switch-case` architecture preventing blocking delays during operation[cite: 1].
- **Serial Parsing Protocol:** Safe buffered parsing for commands enclosed in `<...>` delimiters[cite: 1].

---

## 📐 Kinematics & Link Geometry

The mathematical solver computes joint angles based on the link lengths and mechanical constraints[cite: 1]:

| Joint / Link | Link Length ($mm$) | Angular Limits | Arduino Pin |
| :--- | :--- | :--- | :--- |
| **Base Joint ($\theta_0$)** | $0.00\text{ mm}$[cite: 1] | $0^\circ - 180^\circ$[cite: 1] | Pin 2[cite: 1] |
| **Upperarm Joint ($\theta_1$)** | $146.71\text{ mm}$[cite: 1] | $20^\circ - 100^\circ$[cite: 1] | Pin 3[cite: 1] |
| **Forearm Joint ($\theta_2$)** | $146.71\text{ mm}$[cite: 1] | $0^\circ - 90^\circ$[cite: 1] | Pin 4[cite: 1] |
| **Hand Joint ($\theta_3$)** | $201.10\text{ mm}$[cite: 1] | $0^\circ - 180^\circ$[cite: 1] | Pin 5[cite: 1] |
| **Gripper / Pinza** | $70.00\text{ mm}$[cite: 1] | — | Pin 6[cite: 1] |

### Mathematical Formulation

The geometric solver resolves the end-effector coordinate $(X, Y, Z)$ into joint angles using analytical inverse kinematics[cite: 1]:

$$\theta_1 = \text{atan2}(y, x)$$

$$A = x - l_4 \cos\theta_1 \cos\phi, \quad B = y - l_4 \sin\theta_1 \cos\phi, \quad C = z - l_1 - l_4 \sin\phi$$

$$\theta_3 = \arccos\left(\frac{A^2 + B^2 + C^2 - l_2^2 - l_3^2}{2 l_2 l_3}\right)$$

Where $\phi = \theta_2 + \theta_3 + \theta_4$ defines the end-effector orientation vector[cite: 1].

---

## 🔄 State Machine Architecture

The control routine runs through a structured 5-state loop[cite: 1]:
