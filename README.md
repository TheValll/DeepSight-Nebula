# DeepSight-Nebula

**DeepSight-Nebula is a project combining Robotics & Artificial Intelligence.**

---

## Table of Contents / Sommaire

- [Why the name _DeepSight-Nebula_?](#-why-the-name-deepsight-nebula)
- [Goals of DeepSight-Nebula](#-goals-of-deepsight-nebula)
- [Purpose of this README](#-purpose-of-this-readme)
- [Environment Setup](#-environment-setup)
- [Logbook](#-logbook)

---

## Why the name _DeepSight-Nebula_?

- _DeepSight_: refers to **depth**, reflecting the project's vision system.
- _Nebula_: refers to **nebulas**, celestial objects, to reflect my passion for space.

---

## Goals of DeepSight-Nebula

DeepSight-Nebula investigates **autonomous vision-based manipulation** on a low-cost robotic platform: given an
unstructured scene, the robot must detect a target object, estimate its 3D position, and grasp it without human
intervention.

### Research Objectives

1. **Perception** — Reconstruct the 3D pose of a target object from a passive stereo camera (no Lidar), combining an
   OpenCV-generated depth map with a YOLO-based detector.
2. **Planning & Control** — Plan and execute a collision-aware trajectory with MoveIt2 and ros2_control on a 6-DoF arm
   (Hiwonder xArm ESP32).
3. **Firmware & I/O** — Replace the stock MicroPython firmware with a lean USB-passthrough firmware (C or Rust) to
   minimize command latency on the serial path, with a before/after benchmark.
4. **End-to-end demonstration** — Chain the full perception → planning → actuation loop on a simple target (tennis
   ball) as a first version, then generalize to broader object categories.

### Scope

- **Quasi-static first version** — the target can be mobile during motion (real time correction should will work)
- **Constrained hardware budget** (~400 €).

### Personal Context

This project is part of my preparation for a **PhD application in robotics** (targeted: EPFL, Sept. 2028). It also
serves as a practical sandbox to consolidate skills in ROS2, C++, Rust, computer vision, URDF-based robot modeling,
and embedded firmware.

---

## Purpose of this README

- To summarize and clarify the project's goal for myself and for readers.
- To serve as a **logbook** to document my research and development.

I will record:

- Images
- Diagrams
- Detailed explanations
- Solutions to problems encountered
- ... and maybe even questions I ask myself along the way

I will use **Gemini** to help me rephrase and improve this document throughout the project.

---

## Environment Setup

This project was developed on an **Ubuntu Linux** container in a **Fedora Linux** environment.

### Installation & Execution

#### ROS 2 Packages Documentation :

##### 1. Hiwonder xArm ESP32 URDF Description

This folder contains the URDF description of the Hiwonder xArm ESP32 robot for ROS 2 Humble, including Gazebo models.

Launch the visualization:

```bash
ros2 launch hiwonder_xarm_esp32_description display.launch.xml
```

##### 2. Stereo Camera Calibration

This package contains an application to calibrate a stereo camera.

###### Prerequisites :

- A stereo camera connected via USB.
- A 9x6 Chessboard with 25mm squares.

Configuration: If your chessboard dimensions differ, please modify the configuration in:

```bash
app/src/stereo_camera_calibration/include/stereo_camera_calibration/calibration_node.hpp
```

Run the calibration node:

```bash
ros2 run stereo_camera_calibration calibration_exe
```

###### How it works

1. The script uses VideoCapture (default index: 2) in the capture_frames function.
   Note: You may need to change this index in the code if your camera is on a different port (e.g., /dev/video0).
2. The application will ask you to press **'S'** to take pictures of your chessboard in different positions.
3. Once enough images are captured, it automatically calculates the calibration matrices.
4. A **calibration folder** will be created containing the captured images and the resulting XML calibration file.

---

## Logbook

### 2025-07-22 — Project inception

I started my research by thinking about the design and feasibility of the project.

First, I looked into choosing a robotic arm, but was quickly discouraged by the prices of mid-range models. I then
turned toward sensor-based vision and detection, exploring several options: 2D or 3D Lidar, RGB-D cameras, stereo
cameras, or a combination of these.

Since I planned to use a Raspberry Pi (which I will also use for other projects), several hours of research led me to
a first shortlist:

- An **ELP stereo USB camera** at around €125 ([Amazon
  link](https://www.amazon.fr/ELP-distorsion-Synchronisation-dordinateur-Raspberry/dp/B07FT2GKZS))
- A **TF-Luna Lidar** at around €29 ([Amazon
  link](https://www.amazon.fr/youyeetoo-TF-Luna-Distance-d%C3%A9tection-industrielle/dp/B088BBJ9SQ))

I did not buy them yet, but the figures gave me a first sense of the feasibility and budget. I also ruled out the
Intel RealSense (the best-known stereo option) because its dimensions were too large to mount on a small robotic arm.

**Technical thoughts.** The main difficulty would clearly be the calibration and fusion of the Lidar and camera
streams. I planned to rely on OpenCV, in either Python or C++, with an initial preference for Python to make AI model
integration easier. OpenCV would merge the two camera streams into a usable image; the AI layer would then detect a
target object — probably with **YOLOWorld** as a first model, and a custom-trained model later on once I had time to
build a dataset. The detector would return the `x, y` coordinates of the object; if nothing was in view, the arm would
scan the environment.

**Calibration & precision.** The full system (cameras + Lidar) would need an extrinsic calibration, which I had not
yet mastered. The first prototype idea was to use an **SG90 servo and an Arduino** to orient the camera axis within
roughly 2° of the target — matching the TF-Luna's narrow field of view, so that the Lidar (mounted just below the
camera) could confirm the depth. The `x, y` coordinates would then be recomputed from the calibration. In a later
iteration, the SG90 would be replaced by a motion of the arm's own base.

**Movement and trajectory.** Once `x, y, z` were available, I planned to compute a path for the arm using vectors and
controllers. These early reflections showed me that I still had plenty of room to experiment with what I already owned
before committing to a Raspberry Pi and a robotic arm.

**Estimated budget.**

| Equipment                     | Approx. price |
| ----------------------------- | ------------- |
| ELP stereo camera             | €125          |
| TF-Luna Lidar                 | €29           |
| Robotic arm (entry/mid-range) | €60–200       |
| Raspberry Pi (+ PSU, modules) | €150          |
| **Estimated total**           | **~€400**     |

**Next steps.**

- Get familiar with YOLOWorld and OpenCV.
- Experiment with my current camera (a Logitech StreamCam).
- Find a stable way to mount the camera on the SG90.
- After detecting an object with YOLOWorld, orient the camera axis into the Lidar's ~2° field of view and compute the
  first object coordinates in OpenCV.

**Diagram.** The first milestone is to learn the tools (OpenCV, AI model) using what I already have, then buy the
Lidar and the stereo camera. The goal is to recover the 3D coordinates of an object in the camera's field of view.

![Step 1 diagram](schemas/schema1.png)

---

### 2025-11-03 — Four months of learning and design

It has been four months since my last update — mainly due to lack of time and an intense learning phase.

**Learning and development.** These months were dedicated to building foundations and starting development on the arm.

- I bought the **xArm ESP32** from [Hiwonder](https://www.hiwonder.com/products/xarm-esp32?variant=39662930067543).
- **ROS2 training.** I started learning ROS2 with a [YouTube tutorial](https://www.youtube.com/watch?v=Gg25GfA456o&t),
  going through _nodes_, _publishers_, _subscribers_, _clients_, _servers_, and _actions_.
- **Robot control.** Since the xArm ESP32 has no real manufacturer SDK, I had to reverse-engineer the commands going
  through the USB port. Using **COM8 Monitoring Session**, I sniffed the traffic sent by the stock software and deduced
  the command structure.
- **Mathematics and AI.** In parallel, I started the Coursera specialization [Mathematics for Machine Learning and
  Data Science](https://www.coursera.org/specializations/mathematics-machine-learning-data-science) from
  DeepLearning.AI. As a Data & AI master's student, these courses are essential, especially for the long-term goal of
  training my own model.

**Architecture and design.** My thoughts on hardware and software architecture evolved significantly.

- First ROS architecture sketch — a starting point, not a final solution:

![Architecture diagram](schemas/schema2.png)

- **Dropping the Lidar.** I removed the Lidar from the design. The stereo camera alone is enough to produce a depth
  map via OpenCV, and the Lidar required a very tight alignment (angle < 2°) between camera, gripper, and target. My
  first tests showed the arm was not precise or rigid enough: the combination of weight, latency, and servo accuracy
  caused oscillations during calibration that never converged. I decided to rely solely on the stereo camera.
- **Camera placement.** The stereo camera will sit just below the gripper, fixed on the servo that drives it:

![Camera positioning](schemas/schema3.png)

**Hardware and budget.** On track:

| Equipment              | Price     | Link                                                                            |
| ---------------------- | --------- | ------------------------------------------------------------------------------- |
| xArm ESP32 robotic arm | €229.99   | [Hiwonder](https://www.hiwonder.com/products/xarm-esp32?variant=39662930067543) |
| ELP stereo USB camera  | €125      | [Amazon](https://www.amazon.fr/dp/B07FT2GKZS)                                   |
| Raspberry Pi           | to buy    |                                                                                 |
| **Current total**      | **~€355** |                                                                                 |

**Outline update.** The project now has a clearer spine:

1. **Goal** — autonomously pick up an object (starting with a tennis ball).
2. **AI / vision** — use **YOLOv8** for object detection.
3. **Depth** — use the stereo camera and OpenCV to build a depth map and recover the object's 3D position.
4. **Motion** — learn **URDF** to model the robot and **MoveIt** to plan the trajectory.
5. **Future** — train my own model, potentially with reinforcement learning (either for recognition or for the full
   grasping task).

**Next steps.**

- Wait for the stereo camera to arrive.
- Continue ROS2 training, focusing on URDF and MoveIt via the same YouTube channel.
- Learn 3D modeling or 3D printing to design a custom camera mount. Reach out to my school's innovation club for
  training, or fall back on an existing model like [this camera
  mount](https://makerworld.com/en/models/27135-raspberry-camera-mount?from=search) and contact its creator for
  compatibility.
- Redraw a more detailed global architecture diagram.
- Try to reach Edouard Renard (robotics instructor) to get his opinion before going too deep into the implementation.

---

### 2025-11-08 — First tests with the stereo camera

Progress on the vision side and on modeling the robot.

**Vision setup.** My ELP stereo camera arrived. I wrote a first script, `utils/stereo_camera.py`, that initializes the
camera and displays the left and right video streams side-by-side.

**Architecture simplification.** I simplified the main architecture to make the first version tractable:

![Architecture diagram](schemas/schema4.png)

In this configuration, the arm is not capable of real-time operation — it will have to wait for the target to be
still, and to stay still throughout the motion. I accepted this constraint for V1; real-time trajectory correction
will be a later iteration.

**URDF modeling.** Building the digital twin of the robot in ROS2 was a major and complex step.

- I learned to write a `.xacro` file and to set up a ROS2 package.
- Hiwonder provided me with the `.stp` file of the robot, which I opened in **Fusion 360**.
- I had to sort and group 309 base parts into logical components (gripper, base, limb1, etc.) and create the joints
  between them.
- The key difficulty was that the default exporter,
  [fusion2urdf](https://github.com/syuntoku14/fusion2urdf/tree/master), does not accept Fusion's _as-built_ joints. But
  I could not use simple joints, since the parts were already assembled in the base `.stp` file.
- After three days of searching, I found a [GitHub issue](https://github.com/syuntoku14/fusion2urdf/issues/78)
  describing the same problem. Big thanks to **Colin Fuelberth** ([@Infinite-Echo](https://github.com/Infinite-Echo)),
  who forked the exporter to support as-built joints: [Infinite-Echo/ROS2_fusion2URDF](https://github.com/Infinite-Echo/
  ROS2_fusion2URDF/tree/URDF_Exporter_asBuilt_Support).
- Using his fork, I finally exported a complete ROS2 package with a `.xacro` description of the arm, now in
  `modelisations/robot/hiwonder_xarm_esp32_description`. This URDF does not yet handle gripper opening and closing — I
  will come back to it later. The priority was to get the basics in place and learn Fusion.

**Next steps.**

- Follow a Blender tutorial to later model and 3D-print my own camera mount.
- Move to Docker or a Linux dual-boot for ROS2. Windows has been a constant friction point, especially for opening and
  viewing my `.xacro`.

---

### 2025-11-14 — URDF joints and switch to Docker

I kept working on the URDF model. I described the joints between the arm's limbs with their limits — except for the
gripper, since the _mimic_ joints concept is a bit too much for me right now. I will probably control the gripper
directly via the ESP32.

In the `app/` folder, the first `joint_state_publisher_node` package is functional with install instructions. I
switched to Docker for ROS2 Humble, but I hit a frame-rate problem in RViz / Gazebo simulation that makes the
experience painful.

You can find here a zip with the original `.stp` from Hiwonder and my edited Fusion 360 file, in case modifications
are needed: [Google Drive link](https://drive.google.com/file/d/1qIVWolMBeF4Z5x8Bm8aadgIRzJTUaZLs/view?usp=sharing).

Thanks to this step I can visualize the arm's main joints in RViz. I don't think I will use Gazebo on this project.

![RViz result](schemas/schema5.gif)

**Next steps.**

- Continue the Blender tutorial to later 3D-print my own camera mount.
- Read up on **ros2_control** and **MoveIt**.
- Learn to use **mimic joints** to move the gripper in RViz. I already started creating the corresponding joints in
  Fusion, but the URDF format does not accept closed joint loops.

![Fusion 360 gripper visualization](schemas/schema6.gif)

---

### 2025-12-11 — 3D printing and switch to C++

Big step forward, with major changes on the development environment and mechanical design.

**Modeling and 3D printing.** I finished the Blender tutorial and ran the first test print of the camera mount.

- First attempt — the print came out fine, but a dimension error of a few millimeters prevents the camera from fitting
  into the bracket.
- Correction — I redesigned the part and I am waiting for the new print. Here are some images of the failed prototype:

![Support image](schemas/schema7.jpg)
![Support image](schemas/schema8.jpg)
![Blender sketch](schemas/schema9.png)
![Blender sketch](schemas/schema10.png)

Acknowledgments: thanks to **Bertrand Tech** for his [Blender
tutorial](https://www.youtube.com/watch?v=_5Js5pbvFSw&t), and to [weebzardbbx](https://www.twitch.tv/weebzardbbx) for
the advice and tips that helped me build the mount.

**Environment and language — migration to C++.**

- I decided to develop the core of the project in C++, with a few Python scripts on the side. The goal is twofold:
  better real-time performance, and deepening my mastery of C++ — unlike Python, which I have been using daily for more
  than two years at my work-study company.
- I brushed up on C++ through tutorials, with help from Gemini to unblock some Linux-specific or language-specific
  errors.
- I wrote the first tool, `utils/calibration/stereo_calibration.cpp` (with a CMake build file), which calibrates my
  stereo camera and outputs an XML calibration file — a prerequisite for the rest of the pipeline.

**Next steps.**

- Read up on ros2_control and MoveIt.
- Learn to use mimic joints to move the gripper in RViz.
- Develop a ROS2 node wrapping the camera calibration.
- Develop a C++ node that locates an object in 3D using the YOLO model (already experimented in Python).
- Rewrite the README in the `app/` folder to document the full install procedure.

---

### 2025-12-27 — Math certification and MoveIt crash course

A quick recap of the last few days.

- **Math certification.** I finished the Mathematics for Machine Learning and Data Science specialization ([LinkedIn
  post](https://www.linkedin.com/feed/update/urn:li:activity:7409033718398472192/)). This frees up real time for the
  project.
- **Stereo calibration node.** I built a C++ node that calibrates the stereo camera using a chessboard and outputs the
  calibration matrix.
- **ROS2 Jazzy.** I will probably move from ROS2 Humble to Jazzy, for better alignment with MoveIt and other upstream
  updates. I am also considering buying a paid ROS2 course like [this one on
  Udemy](https://www.udemy.com/course/ros2-for-beginners/?couponCode=DEC_25).
- **MoveIt crash course.** I followed a [quick MoveIt overview video](https://www.youtube.com/watch?v=-xDyxxRiW7M) and
  found the library simpler than I expected. I will need to rebuild my URDF without the gripper — MoveIt can attach one
  cleanly, which would remove the `mimic_joint` problem entirely. Thanks again to Edouard Renard for this course.
- **Real-time re-opens.** From the MoveIt API, it seems I can send both the current position and a _goal_ position
  obtained from YOLO. Real-time control might actually be within reach — for example, letting the arm dodge my hand
  using the depth map.
- **ROCm.** Thanks to my dual-boot, I installed ROCm on my AMD GPU painlessly. A YOLO model now runs with very good
  performance locally.
- **README merge.** I merged this README with the `app/` folder README in the [Environment Setup](#-environment-setup)
  section, in English only.
- **Edouard Renard contact.** I reached out to Edouard Renard to try and set up a call — I have a pile of questions.

**Next steps.**

- **Rethink the ROS2 architecture once more.** Most of the work so far has been learning and design — almost no real
  implementation. But this saves a huge amount of time later and avoids burning five days on a feature that turns out to
  be wrong.
- **Hardware.** The corrected camera mount should be ready soon.
- **Electronics.** Start some electronics courses. With my STI2D (Energy & Environment) background I should not
  struggle to pick it back up.
- **Training.** Buy Edouard Renard's ROS2 courses — the free ones on his channel already helped enormously.

---

### 2026-01-26 — Camera mount done, first hardware-level control library

- **Camera mount printed.** The stereo-camera bracket is finally printed and fitted:

![Support image](schemas/schema11.webp)
![Support image](schemas/schema12.webp)
![Support image](schemas/schema13.webp)
![Support image](schemas/schema14.webp)

- **C++ control library.** I wrote a small C++ library that drives the arm directly at the hardware level.
- **First integrated demo.** Using that library, I wrote a script that performs a basic motion while opening the
  camera stream in parallel. The YOLO integration is the next piece to add:

![Simple motion demo](schemas/schema15.gif)

- **Architecture refactor.** I redesigned my ROS2 architecture taking into account how MoveIt and `ros2_control`
  actually fit together, which makes the overall logic much simpler than the first version:

![New architecture](schemas/schema16.png)

**Next steps.**

- Finalize the bracket installation and the on-arm calibration.
- Proceed with Edouard Renard's paid ROS2 courses.

---

### 2026-03-30 — ROS2 courses completed, firmware pivot

A recap of the past few months of progress.

- **ROS2 courses completed.** I finished Edouard Renard's ROS2 courses covering ROS2 basics, `ros2_control`, and
  MoveIt2. The project will officially move to ROS2 Jazzy instead of Humble.
- **Learning Rust.** I started learning Rust, which I expect to be very useful for the rest of the development —
  firmware in particular.
- **URDF package finalized.** My `hiwonder_xarm_esp32` package is now complete. The robot can be visualized in RViz2
  with correct joints and limits. Because the gripper uses a deformable-parallelogram (parallel) mechanism, URDF does
  not support the resulting closed-loop joint structure. My workaround is a **prismatic joint** that simulates the claw
  opening: it looks unusual visually, but is fully functional. This was a hard step to get right.
- **Hardware connection reconsidered.** While going through the MoveIt course I realized that my current way of
  talking to the hardware is far from optimal. Today, servos are driven by a MicroPython script on the ESP32 controller,
  which introduces significant latency through the chain `command → MicroPython → hex formatting → UART → servos`.
- **Firmware optimization plan.** I plan to rewrite the board's firmware to implement **USB passthrough**. The idea is
  to send the hex commands directly from the host — I already retrieved the original command structure via Thonny.
  Removing the MicroPython layer should bring a significant performance gain, which I intend to quantify with a
  before/after benchmark. I haven't yet decided between C and Rust for this firmware.
- **AI and MoveIt — next big piece.** The next step is to configure the `moveit_config` package with a mock component
  while the new firmware and driver are being built. On the AI side, I will start writing the node that captures frames
  from the calibrated camera, detects the target via YOLO, and forwards a goal pose to MoveIt through topics. I still
  have to work out how to automate the gripper's opening/closing via the MoveIt API.

**Next steps.**

- **Electronics.** Start the basic electronics courses. My STI2D (Energy & Environment) background should make this
  accessible.
- **Training.** Short pause on paid training to practice math, Rust, and to keep the project moving. Afterward I will
  start the DeepLearning.AI ML and DL certifications. The SLAM and Nav2 sections of ROS will come later.

---

### 2026-04-24 — Personal wiki, Rust deep-dive, first research papers

These last weeks were not spent on active development on DeepSight-Nebula, but on long-term learning and on preparing
the foundations for the PhD application.

- **Personal wiki.** I started building a structured personal wiki covering the technical domains I am studying —
  Rust, ROS2, and mathematics for now, with embedded, low-level, electronics, ML and DL planned. Each page synthesizes
  books, courses, videos, and articles in a single reviewable format. The wiki also hosts a spaced-repetition review
  system that drives evening practice sessions (one to two hours), split between new-concept lessons and
  articulation-based drills. The goal is to make the knowledge durable across domains instead of surface-reviewing it
  once.

- **Rust — back to the book.** I am going through the official Rust book chapter by chapter and keeping per-chapter
  notes in the wiki. The foundations (ownership, borrowing, lifetimes, error handling, traits, generics, tests) feel
  solid; the remaining points I want to consolidate are module visibility, `?` vs `unwrap`, and lifetime elision rules.
  This effort feeds directly into the firmware rewrite planned for the xArm controller, where Rust is one of the two
  candidates.

- **Mathematics.** After the DeepLearning.AI specialization, I am keeping a steady math loop — both for its own sake
  and for the upcoming ML and DL certifications. The current focus is to write **intuition pages** for each math concept
  (step-by-step ASCII schemas, physical analogies, no formulas) before resuming practice problems. This slows
  throughput, but it makes each concept something I can re-explain from scratch instead of something I can only
  recognize.

- **First research papers.** I started reading research papers — not only for content, but also to understand how a
  paper is structured. I began with Simon Peyton Jones's _How to write a great research paper_ and Michael Ernst's _How
  to write a technical paper_. This is also why I am rewriting this README in a paper-oriented format, and I plan to
  publish a proper LaTeX paper alongside the repo once the first end-to-end version of DeepSight-Nebula is running.

**Next steps.**

- Resume firmware work (USB passthrough) on the ESP32 controller, once the Rust foundations feel comfortable.
- Finalize the `moveit_config` package with a mock component.
- Write the YOLO → MoveIt goal-pose node.
- Keep the wiki and review system running in parallel with implementation work.
