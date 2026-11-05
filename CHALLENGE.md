# Robotics 2 winter term 26/27 group challenge

Develop a ROS 2 application that enables the Franka Emika Panda to complete six manipulation tasks using perception, motion planning, grasping and task sequencing. Develop and test your methods in simulation for transfer to the physical workspace.

Complete Levels 1-6 in order without resetting the workspace between levels. Starting poses may vary between attempts. A wrist-mounted camera and an external camera overlooking the table are available; use either camera or both. The final task is to assemble a five-part product.

## Workspace labels

| Label | Equipment | Role |
| --- | --- | --- |
| **A0** | Open staging and working area | Starts with fourteen loose parts and a clear tower destination; also supports temporary placements and regrasping directly on the table. |
| **B1-1 / B1-2 / B1-3** | Three sorting trays | Permanent cube / cuboid / cylinder trays. Level 1 uses a temporary colour-routing instruction; tray IDs and physical labels do not change. |
| **C0** | Fixture carrier | Supports the skill fixtures C1-C3 and product assembly nest C4. |
| **C1** | L-shaped fixture | Holds O4-Y after Level 4, until it is reused in Level 6. |
| **C2** | Marked cuboid target | Holds O2-B with its required top/front orientation. |
| **C3** | Round peg socket | Holds O5-1 after Level 5, until it is reused in Level 6. |
| **C4** | Product assembly nest | Locates the movable product base O7-1 while P1 is assembled. |
| **D0 / D1 / D2 / D3** | Drawer case / drawer / handle / guides and stops | Drawer assembly D. O6-1 starts inside closed D1; operate D1 using D2 in Level 6. |

Use free space on A0 for temporary placements and regrasping. Keep the marked tower destination clear until Level 3. Both cameras are available throughout; calibration and fixed-fixture coordinates may be provided.

## Workpiece labels

| IDs | Workpieces | Identification |
| --- | --- | --- |
| **O1-R / O1-G / O1-B** | Three cubes | Red, green and blue; 40 × 40 × 40 mm. |
| **O2-R / O2-G / O2-B** | Three cuboids | Red, green and blue; 50 × 30 × 40 mm. White top mark and black front mark. |
| **O3-R / O3-G / O3-B** | Three cylinders | Red, green and blue; diameter 36 mm, height 40 mm. |
| **O4-Y / O4-V** | Two L-shaped pieces | Yellow and violet; 60 × 50 mm L-footprint, 30 mm thick. White top mark. |
| **O5-1** | Round peg with gripping head | Orange; circular shaft, 20 mm diameter and 20 mm long. |
| **O6-1** | D-shaped peg with gripping head | Turquoise; shaft has one flat side. A head mark identifies its direction. |
| **O7-1** | Stepped product base | Light grey; L-shaped recess between two raised peg bosses. |
| **O8-1** | Product cover | Dark grey; integral top grip bar, one round hole and one D-shaped hole. |

There are **15 movable parts / 8 geometries**. Fourteen start separately on A0; O6-1 starts head-down, shaft-up inside the closed drawer. O5-1 also starts head-down, shaft-up. Cylinders start upright; cubes, marked cuboids and L-pieces start in stable poses. O7-1 and O8-1 start right-side up. No part is added, removed or replaced between levels.

## Rules applying to all levels

- **Complete the levels in order.** Each level starts from the state left by the previous level. Do not reset, respawn, reload or manually rearrange the workspace between levels.
- **Run autonomously through ROS 2.** After the start signal, report every human intervention. No workpiece poses are supplied. Use observations from either or both cameras and your action history; privileged simulator state, generated scene files, instructor files and random seeds are forbidden as object-pose inputs.
- **Preserve parts and completed placements.** All fifteen parts are present from the beginning. Leave unrelated parts in stable positions. Keep completed results intact unless a later task explicitly reuses them: Level 2 re-sorts the cubes; Level 6 removes O4-Y from C1 and O5-1 from C3. The Level 3 tower and O2-B in C2 must remain intact through the finish.
- **Keep equipment fixed and travel clear.** Free A0 surface may support temporary placement and regrasping. Keep the tower destination clear until Level 3 and leave drawer travel unobstructed throughout. Avoid unintended collisions, forceful wedging and losing parts; intended grasping, support, insertion and drawer contacts are allowed.
- **Release and verify stability.** A checkpoint counts only after all relevant parts have been released and remain stable without gripper support for at least two seconds. Tray and marked-area placements must fit inside their boundaries; parts may extend above tray rims. Tower layers must be centred over their supports, with no contact to other equipment.
- **Demonstrate the required operations.** Record the Level 4 release and change of opposing gripping faces. Keep D1 closed and O6-1 inside it through Level 5. In Level 6, open D1 using D2, retrieve O6-1 and close D1 again. A checkpoint remains credited when its result is legitimately reused later.
- **Use physical grasps and seated joints.** Parts must be held by the simulated or physical gripper. Artificial attachment, teleportation and physics/state edits are forbidden; MoveIt planning attachments may represent a real grasp. P1 must stand assembled in C4 without gripper support; no lift test is required for its gravity-seated joints.

## Level 1 - Locate, pick and place the cubes

**Start:** The complete fifteen-part inventory is present. Fourteen parts are on A0; O6-1 is inside the closed drawer. Locate O1-R, O1-G and O1-B from observations.

**Your task:** Grasp and transport the three cubes according to this colour-routing instruction. Keep the other parts in place and leave the tower destination clear.

| Workpiece | Destination |
| --- | --- |
| O1-R / red cube | B1-1 |
| O1-G / green cube | B1-2 |
| O1-B / blue cube | B1-3 |

Tray IDs and their permanent shape labels stay unchanged. The mixed tray contents are intentional preparation for Level 2.

**Completed when:** All three cubes are stable inside their assigned trays. The other twelve parts remain present; the drawer is closed.

**Focus:** Cube localization, coordinate transforms, grasp planning and gripper control.

## Level 2 - Perceive and sort by shape

**Start:** Continue from Level 1. The cubes are distributed across the trays; all six cuboids and cylinders remain on A0. The other parts retain their current positions.

**Your task:** Use one or both cameras to identify the nine O1/O2/O3 parts and determine their current poses. Sort them by shape, regardless of colour:

| Workpieces | Destination |
| --- | --- |
| O1-R / O1-G / O1-B - cubes | B1-1 |
| O2-R / O2-G / O2-B - cuboids | B1-2 |
| O3-R / O3-G / O3-B - cylinders | B1-3 |

Move the green and blue cubes out of their Level 1 trays. A correctly placed part may stay where it is. Leave O4-O8 available and keep the tower destination clear.

**Completed when:** Each tray contains its three correct parts. No O1/O2/O3 part remains on A0 or in the wrong tray. D1 remains closed.

**Focus:** Pose estimation, shape classification and action ordering. RGB and depth are available; using depth is optional.

## Level 3 - Select and build the order tower

**Start:** Continue from the shape-sorted trays. The tower destination on A0 is clear; the remaining workpieces stay in the workspace.

**Your task:** Retrieve the requested parts from their trays and build this three-layer tower at the marked destination on A0:

| Layer / build order | Required workpiece |
| --- | --- |
| 1 / bottom, resting on the table | O1-R / red cube |
| 2 / middle, white top mark upward | O2-G / green cuboid |
| 3 / top, cylinder axis vertical | O3-B / blue cylinder |

Centre each layer over the one below it. The cuboid and cylinder may overhang their supports because their nominal widths differ; their centre projections must align within 5 mm in each horizontal axis. Release after each placement.

**Completed when:** The complete 120 mm nominal-height tower stands unsupported for two seconds, inside its marked destination. All non-target parts remain stable; keep this tower intact through Levels 4-6.

**Focus:** Selective retrieval, stacking, centre alignment and stability verification.

## Level 4 - Orient, regrasp and place

**Start:** Continue with the tower intact. O4-Y and O4-V are still on A0. O2-B is in B1-2; C1 and C2 are empty.

**Your task:** Complete both operations:

1. Pick the yellow L-piece **O4-Y** and fit it into **C1**, with the white top mark upward. Leave the violet L-piece O4-V as the distractor on A0.
2. Retrieve the blue cuboid **O2-B**. Release it on free, level surface within **A0**. Regrasp it using a **different pair of opposing side faces**, switching between its nominal 30 mm and 50 mm gripping directions. Place it in **C2**, white top upward and black front mark facing the target arrow.

A wrist rotation without releasing and changing gripping faces does not count.

**Completed when:** O4-Y is seated in C1, O2-B is correctly oriented in C2 and the release/regrasp has been demonstrated. The tower remains intact and D1 closed.

**Focus:** Orientation constraints, grasp changes and preservation of completed placements.

## Level 5 - Reorient and insert the round peg

**Start:** Continue from Level 4. O5-1 is still on A0, resting on its head with the shaft upward. C3 is empty. O6-1 stays inside the closed drawer.

**Your task:** Grasp the orange round peg **O5-1** by its head, reorient its shaft downward and insert it into the round socket **C3**. Use free A0 surface for an intermediate release or regrasp if needed.

Approach in alignment with the socket, control the insertion and verify that the head underside reaches the socket's top surface. Leave the peg accessible for withdrawal in Level 6. Do not open the drawer or use O6-1 at this level.

**Completed when:** O5-1 is fully seated in C3 and stable after release. A peg balanced on the rim does not count. The tower, C1 placement and C2 placement remain intact.

**Focus:** Out-of-plane reorientation, controlled insertion and seating verification.

## Level 6 - Assemble product P1

**Start:** Continue from Level 5. O4-Y is in C1, O5-1 is in C3, and O7-1 and O8-1 remain on A0. O6-1 is still in closed D1. The product nest C4 is empty.

**Your task:** Build **P1, the pinned locator cassette**, as one five-part assembly. Use the target product illustration supplied with the challenge. Retrieve and reuse parts from earlier checkpoints:

| Assembly operation | Source and required result |
| --- | --- |
| Locate O7-1 / stepped base | A0 to C4; match the nest's orientation mark. |
| Fit O4-Y / yellow L-piece | Withdraw from C1; seat in the base's L-recess, white top upward. |
| Place O8-1 / cover | A0 onto the base bosses; grip bar upward and both holes aligned. |
| Insert O5-1 / round peg | Withdraw from C3; insert through the cover's round hole into the base. |
| Retrieve and insert O6-1 / D-peg | Open D1 using D2; retrieve the peg, turn its shaft downward and align both D-profiles before insertion. |

Seat both peg heads against the cover and close D1 after retrieval. C1 and C3 may now be empty. Keep the Level 3 tower, O2-B in C2, remaining sorted parts and violet distractor intact.

**Completed when:** All five P1 parts are correctly fitted together in C4 and stable after release; both peg heads are seated, D1 is closed and the protected earlier results are intact.

**Focus:** Multi-part assembly, dependency planning, extraction of previously inserted parts, keyed insertion, drawer operation and recovery.

## P1 target product

P1 is a gravity-seated, five-part locator cassette, assembled in C4: **O7-1 base + O4-Y insert + O8-1 cover + O5-1 round peg + O6-1 D-shaped peg**. The yellow L-piece fits the base recess. The cover rests on the raised bosses and retains the insert underneath. Each peg passes through a matching cover hole into the corresponding base bore; align the D-shaped flat through both parts.

The top grip bar belongs to O8-1; it is not an additional workpiece. C4 is a fixed nest, not part of the product. Pins are slip-fit and gravity-seated: do not infer a positively locked joint or lift the completed product as an acceptance test.

![P1 target assembly](assets/product-P1.png)
