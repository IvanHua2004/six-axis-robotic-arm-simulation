# six-axis-robotic-arm-simulation
RVIz
\documentclass[12pt,a4paper]{article}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage{amsmath, amssymb, amsfonts}
\usepackage{graphicx}
\usepackage{listings}
\usepackage{xcolor}
\usepackage{hyperref}
\usepackage{geometry}
\geometry{margin=2.5cm}

\definecolor{codegreen}{rgb}{0,0.6,0}
\definecolor{codegray}{rgb}{0.5,0.5,0.5}
\definecolor{codepurple}{rgb}{0.58,0,0.82}
\definecolor{backcolour}{rgb}{0.95,0.95,0.92}

\lstdefinestyle{cppstyle}{
    backgroundcolor=\color{backcolour},   
    commentstyle=\color{codegreen},
    keywordstyle=\color{magenta},
    numberstyle=\tiny\color{codegray},
    stringstyle=\color{codepurple},
    basicstyle=\ttfamily\footnotesize,
    breakatwhitespace=false,         
    breaklines=true,                 
    captionpos=b,                    
    keepspaces=true,                 
    numbers=left,                    
    numbersep=5pt,                  
    showspaces=false,                
    showstringspaces=false,
    showtabs=false,                  
    tabsize=2
}

\lstset{style=cppstyle}

\title{Six‑Axis Robotic Arm Simulation with Inverse Kinematics in ROS2 and RViz}
\author{Project Documentation}
\date{\today}

\begin{document}

\maketitle

\begin{abstract}
This document describes a complete ROS2 simulation of a six‑axis robotic arm (base yaw, shoulder pitch, elbow pitch, wrist pitch, and two prismatic fingers). 
An inverse kinematics solver using a Jacobian matrix and damped least squares drives the arm to follow a 3D target moved interactively in RViz. 
The kinematic centre of the claw reaches the target with sub‑millimetre precision, and the orientation is decoupled to ensure the claw points exactly toward the target. 
The URDF model is fully integrated, and all joints are continuous, allowing unlimited rotation.
\end{abstract}

\section{Introduction}
Robotic arm simulation is an essential tool for testing control algorithms before deployment on real hardware. 
This project implements a six‑axis manipulator in the ROS2 ecosystem, using RViz for visualisation and \texttt{interactive\_markers} for user input. 
The inverse kinematics (IK) solver is based on the Jacobian transpose method with damping, providing robust convergence even near singularities.

\section{Robot Kinematics}
The robot has four revolute joints (base yaw, shoulder pitch, elbow pitch, wrist pitch) and two prismatic fingers. 
The coordinate frames follow the standard Denavit–Hartenberg convention, with all revolute axes aligned as follows:
\begin{itemize}
    \item Base yaw: rotation about world $Z$ axis.
    \item Shoulder, elbow, wrist: rotations about the local $Y$ axis (pitch).
\end{itemize}
The geometry is defined by three length parameters:
\begin{align}
    L_1 &= 0.4\ \text{m} \quad\text{(upper arm)}\\
    L_2 &= 0.45\ \text{m} \quad\text{(forearm)}\\
    d_c &= 0.12\ \text{m} \quad\text{(wrist to claw centre, tunable)}
\end{align}
The forward kinematics of the claw centre $\mathbf{p} = (x, y, z)^T$ are computed as:
\begin{align}
    \mathbf{p}_{\text{local}} &= 
    \begin{pmatrix}
        L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2) + d_c\cos(\theta_1+\theta_2+\theta_3)\\
        0\\
        L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2) + d_c\sin(\theta_1+\theta_2+\theta_3)
    \end{pmatrix},\\
    \begin{pmatrix} x \\ y \\ z \end{pmatrix} &=
    \begin{pmatrix}
        \cos\theta_0 & -\sin\theta_0 & 0\\
        \sin\theta_0 & \cos\theta_0 & 0\\
        0 & 0 & 1
    \end{pmatrix}
    \mathbf{p}_{\text{local}} +
    \begin{pmatrix} x_{\text{base}} \\ 0 \\ 0 \end{pmatrix},
\end{align}
where $\theta_0$ is the base yaw angle, $\theta_1$ shoulder pitch, $\theta_2$ elbow pitch, $\theta_3$ wrist pitch, and $x_{\text{base}} = 0.05$ m is the fixed offset of the shoulder from the world origin.

\section{Inverse Kinematics Solver}
The IK problem is to find joint angles $\boldsymbol{\theta} = (\theta_0,\theta_1,\theta_2,\theta_3)^T$ such that the claw centre $\mathbf{p}(\boldsymbol{\theta})$ equals a desired target $\mathbf{p}_d$. 
Because the system is overactuated (3 position constraints, 4 joints), we use a Jacobian‑based method with a damping factor.

\subsection{Jacobian Matrix}
The Jacobian $\mathbf{J} \in \mathbb{R}^{3\times4}$ relates joint velocities to the velocity of the claw centre:
\[
\dot{\mathbf{p}} = \mathbf{J} \, \dot{\boldsymbol{\theta}}.
\]
Its columns are:
\begin{align}
\mathbf{J}_{:,0} &= \frac{\partial\mathbf{p}}{\partial\theta_0} = 
\begin{pmatrix}
 -x_{\text{local}}\sin\theta_0 \\
  x_{\text{local}}\cos\theta_0 \\
  0
\end{pmatrix},\\
\mathbf{J}_{:,1} &=
\begin{pmatrix}
\cos\theta_0\, \frac{\partial x_{\text{local}}}{\partial\theta_1}\\[2pt]
\sin\theta_0\, \frac{\partial x_{\text{local}}}{\partial\theta_1}\\[2pt]
\frac{\partial z_{\text{local}}}{\partial\theta_1}
\end{pmatrix},\;
\mathbf{J}_{:,2} =
\begin{pmatrix}
\cos\theta_0\, \frac{\partial x_{\text{local}}}{\partial\theta_2}\\[2pt]
\sin\theta_0\, \frac{\partial x_{\text{local}}}{\partial\theta_2}\\[2pt]
\frac{\partial z_{\text{local}}}{\partial\theta_2}
\end{pmatrix},\;
\mathbf{J}_{:,3} =
\begin{pmatrix}
\cos\theta_0\, \frac{\partial x_{\text{local}}}{\partial\theta_3}\\[2pt]
\sin\theta_0\, \frac{\partial x_{\text{local}}}{\partial\theta_3}\\[2pt]
\frac{\partial z_{\text{local}}}{\partial\theta_3}
\end{pmatrix},
\end{align}
where $(x_{\text{local}}, z_{\text{local}})$ are the coordinates in the arm’s local plane.

\subsection{Damped Least‑Squares Update}
Given a position error $\mathbf{e} = \mathbf{p}_d - \mathbf{p}(\boldsymbol{\theta})$, the joint update is:
\[
\Delta\boldsymbol{\theta} = \alpha\; \mathbf{J}^T \left( \mathbf{J}\mathbf{J}^T + \lambda \mathbf{I} \right)^{-1} \mathbf{e},
\]
where $\alpha = 0.3$ is the step size and $\lambda = 0.01$ is the damping factor (prevents numerical issues near singularities). 
The iteration continues until $\|\mathbf{e}\| < 10^{-4}$ m or a maximum of 500 iterations.

\subsection{Orientation Decoupling}
After the centre position converges, the claw orientation is set independently. 
The desired world pitch angle $\phi_{\text{des}}$ is the direction from the wrist to the target:
\[
\phi_{\text{des}} = \operatorname{atan2}( \Delta z,\; \sqrt{\Delta x^2+\Delta y^2} ).
\]
Because the claw world angle equals $\theta_1+\theta_2+\theta_3$, the required wrist angle is:
\[
\theta_3 = \phi_{\text{des}} - (\theta_1+\theta_2) + \delta_{\text{urdf}},
\]
where $\delta_{\text{urdf}}$ is a constant offset to compensate for the visual orientation of the claw in the URDF.

\section{Software Architecture}
The main node \texttt{ik\_solver.cpp}:
\begin{itemize}
    \item Subscribes to \texttt{/target\_position} (\texttt{geometry\_msgs/Point}) – the 3D target.
    \item Provides an interactive marker server (\texttt{ik\_target}) – users can drag the marker in RViz.
    \item Runs the IK solver every time a new target is received.
    \item Publishes the computed joint angles to \texttt{/joint\_states} (\texttt{sensor\_msgs/JointState}).
    \item Includes a capture detection: when the claw centre is within 1 cm of the target, the marker turns green and motion stops.
\end{itemize}
The robot model is loaded by \texttt{robot\_state\_publisher} using the URDF file. 
All joints are defined as \texttt{continuous} to allow unlimited rotation.

\section{Calibration of Visual Model}
The kinematic centre $\mathbf{p}$ is the point that the IK drives to the target. 
For the ball to appear visually between the fingers, two parameters must be adjusted:
\begin{itemize}
    \item \texttt{CLAW\_CENTER\_DIST} – distance from wrist to the kinematic centre.
    \item \texttt{CLAW\_URDF\_OFFSET} – extra rotation added to $\theta_3$ to align the URDF visual.
\end{itemize}
Calibration procedure:
\begin{enumerate}
    \item Place the target at a known position (e.g. $x=0.5$, $y=0$, $z=0$).
    \item Observe the ball’s position relative to the claw fingers in RViz.
    \item If the ball is ahead of the fingers, decrease \texttt{CLAW\_CENTER\_DIST}; if behind, increase it.
    \item If the claw points sideways, adjust \texttt{CLAW\_URDF\_OFFSET} in steps of $\pi/2$.
    \item Rebuild and relaunch until the ball is perfectly centred for all reachable positions.
\end{enumerate}

\section{Launch and Usage}
\subsection{Building}
\begin{lstlisting}[language=bash]
cd ~/ros2_ws
colcon build --packages-select my_robot
source install/setup.bash
\end{lstlisting}

\subsection{Running}
\begin{lstlisting}[language=bash]
ros2 launch my_robot display.launch.py use_gui:=false
\end{lstlisting}
RViz opens with the robot and a yellow interactive sphere. 
Drag the sphere in the XY plane (or use the Z‑axis arrow) to move the target; the arm follows automatically. 
When the claw centre is within 1 cm, the marker turns green and the arm stops.

\subsection{Joint Names}
The published joint state message uses the following names:
\begin{verbatim}
base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch,
claw_left_joint, claw_right_joint
\end{verbatim}
The prismatic fingers are not used in the IK (they remain at 0), but can be controlled separately.

\section{Results}
The solver achieves a position error consistently below $10^{-4}$ m for all reachable targets. 
The capture distance of 1 cm provides a stable dead‑zone, preventing jitter. 
Thanks to the damping factor, the arm converges smoothly even near singular configurations (e.g. fully extended).

\section{Conclusion}
This ROS2 package demonstrates a complete simulation pipeline for a six‑axis robotic arm with a robust IK solver. 
The decoupled orientation ensures the claw always points toward the target. 
Calibration constants allow easy adaptation to any URDF model. 
The code is ready for integration with hardware or further extension, such as adding velocity limits or trajectory smoothing.

\appendix
\section{URDF Snippet}
The essential joints in the URDF must be defined as:
\begin{lstlisting}[language=xml]
<joint name="base_yaw" type="continuous">
  <parent link="world"/>
  <child link="base_link"/>
  <axis xyz="0 0 1"/>
</joint>
<joint name="shoulder_pitch" type="continuous">
  <parent link="base_link"/>
  <child link="upper_arm"/>
  <axis xyz="0 1 0"/>
</joint>
<!-- similarly for elbow_pitch and wrist_pitch -->
\end{lstlisting}

\section{Key Parameters in \texttt{ik\_solver.cpp}}
\begin{itemize}
    \item \texttt{L1 = 0.4}, \texttt{L2 = 0.45} – link lengths.
    \item \texttt{CLAW\_CENTER\_DIST = 0.12} – distance wrist → claw centre (tune to your URDF).
    \item \texttt{CLAW\_URDF\_OFFSET = 0.0} – visual orientation offset (add $\pm\pi/2$ as needed).
    \item \texttt{JOINT\_SPEED = 3.0} – maximum joint speed (rad/s).
    \item \texttt{CAPTURE\_DIST = 0.01} – dead‑zone radius (metres).
\end{itemize}

\section*{Acknowledgements}
This work uses the ROS2 framework and the Eigen linear algebra library. 
The interactive marker server is part of the \texttt{interactive\_markers} ROS2 package.

\end{document}
