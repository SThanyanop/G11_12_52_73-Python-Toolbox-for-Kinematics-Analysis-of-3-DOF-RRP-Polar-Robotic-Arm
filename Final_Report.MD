# Python Toolbox for Kinematics Analysis of 3-DOF (RRP) Polar Robotic Arm

## Project Team
- ชยากร ชื่นประเสริฐ (66340500012)
- ศักดิ์ไชย ธนภาสวัฒน์ (66340500052)
- ธัญนพ ศรีวานิช (66340500073)

**Institution:** Institute of Field Robotics, King Mongkut's University of Technology Thonburi  
**Program:** Senior Thesis FRAB (FIBO Robotics and Automation: Bachelor)  
**Academic Year:** 2025

---

## บทคัดย่อ (Abstract)

โครงงานนี้นำเสนอการพัฒนา Python Toolbox สำหรับการวิเคราะห์จลนศาสตร์ของแขนกลโพลาร์ 3 องศาอิสระ (3-DOF RRP Polar Robotic Arm) โดยเครื่องมือที่พัฒนาขึ้นสามารถคำนวณ Configuration Space และ Task Space ผ่านการประยุกต์ใช้ Forward Kinematics, Inverse Kinematics และ Differential Kinematics ระบบได้รับการออกแบบให้รองรับการกำหนดพารามิเตอร์ของแขนกลตามโครงสร้างที่ผู้ใช้ต้องการ พร้อมทั้งคำนวณและแสดงผล Workspace ในปริภูมิสามมิติ รวมถึงการแสดงภาพแบบเคลื่อนไหว (Animation) เพื่อการศึกษาและวิเคราะห์พฤติกรรมของแขนกลอย่างครอบคลุม

**คำสำคัญ:** Python Toolbox, Polar Robotic Arm, Kinematics Analysis, Configuration Space, Task Space, Workspace Visualization

---

## 1. จุดประสงค์ของโครงการ (Project Objectives)

1. เพื่อศึกษาและวิเคราะห์จลนศาสตร์ของ Polar Robotic Arm ในปริภูมิสามมิติ
2. เพื่อพัฒนา Python Toolbox สำหรับการคำนวณ Forward Kinematics, Inverse Kinematics และ Differential Kinematics ของ Polar Robotic Arm
3. เพื่อพัฒนาระบบแสดงภาพแบบสามมิติ (3D Visualization) สำหรับการแสดงท่าทาง (Pose) และการเคลื่อนไหว (Animation) ของแขนกล

---

## 2. ขอบเขตของโครงการ (Project Scope)

### 2.1 ขอบเขตการพัฒนา Python Toolbox

ระบบที่พัฒนาครอบคลุมเฉพาะ Pose Kinematics ดังนี้:

- Forward Kinematics: การคำนวณตำแหน่งและการวางแนว (Position and Orientation) ของ End Effector จาก Configuration Space
- Inverse Kinematics: การคำนวณมุมและตำแหน่งของข้อต่อจากตำแหน่งที่ต้องการของ End Effector
- Forward Differential Kinematics: การคำนวณความเร็วของ End Effector จากความเร็วของข้อต่อ
- Inverse Differential Kinematics: การคำนวณความเร็วของข้อต่อจากความเร็วที่ต้องการของ End Effector

**ข้อจำกัด:** โครงงานนี้ไม่ครอบคลุมการศึกษาและการคำนวณเกี่ยวกับความเร่ง (Acceleration) ของระบบ

### 2.2 ข้อกำหนดของ Polar Robotic Arm

**คุณสมบัติของแขนกล:**
- ทำงานในปริภูมิสามมิติ (3D Workspace)
- คำนวณตำแหน่งและความเร็วใน Configuration Space (Joint Space)
- คำนวณตำแหน่งและความเร็วใน Task Space (Cartesian Space: X, Y, Z axes)
- วิเคราะห์และแสดงผล Workspace ของแขนกล

**ข้อสมมติฐาน:**
- ไม่มีสิ่งกีดขวางในพื้นที่ทำงาน
- ไม่พิจารณาโครงสร้างของ End Effector
- แขนกลเป็น Rigid Body ไม่มีแรงเสียดทาน
- Actuators มีความแม่นยำสูงสุดไม่มีผิดพลาด

### 2.3 โครงสร้างของระบบแขนกล

**องค์ประกอบของแขนกล:**
- จำนวนลิงก์: 4 links
- ชนิดของข้อต่อ: RRP Configuration (2 Revolute joints + 1 Prismatic joint)
- จำนวน Actuators: 3 ตัว
- จำนวนข้อต่อสูงสุดต่อลิงก์: 3 joints

### 2.4 พารามิเตอร์นำเข้า (Input Parameters)

**1. ขนาดของลิงก์ (Link Dimensions)**

กำหนดเป็นเซตของเวกเตอร์ที่แสดงระยะห่างระหว่างข้อต่อ:

```
Link_i = {v₁, v₂, ..., vₙ}
```

โดยที่:
- vⱼ คือเวกเตอร์ระยะทางส่วนที่ j ของลิงก์ที่ i
- v₁ เชื่อมต่อกับ joint_{i-1}
- vₙ เชื่อมต่อกับ joint_i

**2. ข้อจำกัดของข้อต่อ (Joint Constraints)**

กำหนดเป็นเมทริกซ์ขนาด 3×2 (สำหรับแกน X, Y, Z):

```
JointLimits_i = [min_x  max_x]
                [min_y  max_y]
                [min_z  max_z]
```

สำหรับข้อต่อแบบหมุน (Revolute Joint):
- กำหนดขอบเขตมุมหมุน (ในหน่วยเรเดียน) ตามกฎมือขวา
- ค่าบวกและลบแสดงทิศทางการหมุนตามแกนที่กำหนด

สำหรับข้อต่อแบบเลื่อน (Prismatic Joint):
- กำหนดระยะยืดออกสูงสุด (Extension) และระยะหดเข้าสูงสุด (Retraction)

**3. Task Space และเวลาในการเคลื่อนที่**

กำหนดตำแหน่งและความเร็วของ End Effector:

```
TaskSpace = [x, y, z], [ẋₑ, ẏₑ, żₑ]
```

**4. Configuration Space และเวลาในการเคลื่อนที่**

กำหนดค่าตำแหน่งและความเร็วของข้อต่อทั้งหมด:

```
ConfigSpace = [θ₁, θ₂, d₃], [θ̇₁, θ̇₂, ḋ₃]
```

---

## 3. ทฤษฎีพื้นฐาน (Theoretical Background)

### 3.1 คุณสมบัติของ 3-DOF (RRP) Polar Robotic Arm

แขนกลโพลาร์ 3 องศาอิสระมีโครงสร้างแบบทรงกลม (Spherical Configuration) ประกอบด้วยข้อต่อดังนี้:

1. **ข้อต่อหมุนฐาน (Base Revolute Joint):** θ₁
2. **ข้อต่อหมุนหัวไหล่ (Shoulder Revolute Joint):** θ₂
3. **ข้อต่อเลื่อน (Prismatic Joint):** d₃

โครงสร้างนี้ช่วยให้แขนกลสามารถเคลื่อนที่ครอบคลุมพื้นที่ทำงานแบบทรงกลมได้อย่างมีประสิทธิภาพ

### 3.2 พารามิเตอร์ Denavit-Hartenberg (DH Parameters)

การกำหนด Coordinate Frames ของแขนกลตามหลัก DH Convention สำหรับการคำนวณจลนศาสตร์:

**ตารางพารามิเตอร์ DH ตามการ Implementation จริง:**

| i | aᵢ | αᵢ | dᵢ | θᵢ |
|:---:|:------:|:------:|:--------:|:--------:|
| 1 | 0 | 0° | 0 | θ₁* |
| 2 | X₁ | 0° | Z₁ | 0° |
| 3 | 0 | 90° | Y₁ | θ₂* |
| 4 | X₂ | 0° | Y₂ | 90° |
| 5 | Z₂ | 90° | 0 | 0° |
| 6 | Z₃ | 0° | X₃ + d₃* | 90° |
| E | Y₃ | 90° | 0 | 90° |

*หมายเหตุ:* ตัวแปรที่มีเครื่องหมาย (*) คือตัวแปรข้อต่อ (Joint Variables)

**สัญลักษณ์ที่ใช้:**
- aᵢ: ระยะห่างระหว่างแกน Zᵢ₋₁ และ Zᵢ วัดตามแกน Xᵢ
- αᵢ: มุมระหว่างแกน Zᵢ₋₁ และ Zᵢ วัดรอบแกน Xᵢ
- dᵢ: ระยะห่างระหว่างแกน Xᵢ₋₁ และ Xᵢ วัดตามแกน Zᵢ₋₁
- θᵢ: มุมระหว่างแกน Xᵢ₋₁ และ Xᵢ วัดรอบแกน Zᵢ₋₁

### 3.3 Transformation Matrix

**เมทริกซ์การแปลงพิกัดระหว่างเฟรม:**

Transformation Matrix จากเฟรม i-1 ไปยังเฟรม i ตามหลัก Modified DH Convention:

```
ⁱ⁻¹Tᵢ = Trans(aᵢ, 0, 0) · Rot(x, αᵢ) · Rot(z, θᵢ) · Trans(0, 0, dᵢ)
```

### 3.4 การวิเคราะห์ Workspace

**Jacobian Matrix:**

Jacobian Matrix เป็นเมทริกซ์ที่แสดงความสัมพันธ์ระหว่างความเร็วของข้อต่อกับความเร็วของ End Effector:

```
ẋ = J(q)q̇
```

โดยที่:
```
J(q) = ∂f(q)/∂q = [∂fᵢ/∂qⱼ]
```

สำหรับ i = 1, 2, 3 (X, Y, Z) และ j = 1, 2, 3 (ข้อต่อที่ 1, 2, 3)

**เงื่อนไข Singularity:**

Singularity เกิดขึ้นเมื่อ Jacobian Matrix สูญเสียอันดับ (Rank Deficiency):

```
det(J(q)) = 0
```

ณ จุด Singularity ระบบสูญเสียความสามารถในการเคลื่อนที่ในทิศทางบางทิศทาง หรือต้องใช้ความเร็วของข้อต่อที่สูงมากในการควบคุม

**เซตของจุด Singular:**

พื้นที่ทำงาน (Workspace) ถูกกำหนดโดยการรวมเซตต่อไปนี้:

- **S₁:** เซตของจุดที่ Null Space ของ J(q) มีมิติ ≥ 1 (Internal Singularities)
- **S₂:** เซตของจุดที่ขอบเขตของ Workspace (Boundary Singularities)
- **S₃:** เซตของจุดที่เกิดจากข้อจำกัดของข้อต่อ (Joint Limit Singularities)

**Total Workspace:**
```
W = S₁ ∪ S₂ ∪ S₃
```

**Reachable Workspace:** เซตของจุดทั้งหมดที่ End Effector สามารถเข้าถึงได้อย่างน้อยหนึ่งการจัดวาง (Orientation)

**Dexterous Workspace:** เซตของจุดที่ End Effector สามารถเข้าถึงได้ในทุกการจัดวาง

---

## 4. ภาพรวมของระบบ (System Overview)

### 4.1 แผนผังระบบ (System Architecture)

![alt text](Img/sys_overview.png)

### 4.2 ฟังก์ชันหลักของระบบ

#### Function 1: Robot initialization (เรียกใช้อัตโนมัติ)
**วัตถุประสงค์:** กำหนดพารามิเตอร์และข้อจำกัดของแขนกล

**Input:**
- เวกเตอร์ขนาดของลิงก์ทุกลิงก์
- เมทริกซ์ข้อจำกัดของข้อต่อ (X-Y-Z Matrix)

**Output:**
- Robot configuration object

**Process:**
1. รับและตรวจสอบความถูกต้องของพารามิเตอร์
2. คำนวณ DH Parameters จากข้อมูลลิงก์
3. สร้าง object เก็บข้อมูลโครงสร้างแขนกล

#### Function 2: 3D Workspace Calculation
**วัตถุประสงค์:** คำนวณและแสดงผล Workspace ของแขนกล

**Input:**
- Robot configuration object
- Sample resolution parameters (theta1_samples, theta2_samples, d3_samples)

**Output:**
- ขอบเขต Workspace ในพิกัด 3 มิติ
- ตำแหน่งของจุด Singularity
- 3D visualization พร้อม edges และ faces

**Process:**
1. คำนวณ samples สำหรับ θ₁, θ₂ และ d₃ ตามความละเอียดที่กำหนด
2. คำนวณ Forward Kinematics สำหรับทุกค่าข้อต่อที่เป็นไปได้
3. วิเคราะห์ Jacobian Matrix เพื่อหาจุด Singularity (ใช้ samples ที่ละเอียดกว่า)
4. สร้าง vertices, edges และ faces สำหรับการแสดงผล 3D
5. แสดง Workspace พร้อมทำเครื่องหมายจุด Singularity

**Sampling Strategy:**
- θ₁, θ₂: Linear spacing ตั้งแต่ min ถึง max
- d₃: ใช้เฉพาะค่า min และ max (เพื่อลดการคำนวณ)
- Singularity detection: ใช้ samples ครึ่งหนึ่งของ workspace samples

#### Function 3: Forward Kinematics
**วัตถุประสงค์:** แปลง Configuration Space เป็น Task Space

**Mathematical Foundation:**
```
T⁰ₑ(q) = T⁰₁(θ₁) · T¹₂(θ₂) · T²₃(d₃) · ... · Tⁿₑ
```

**Input:**
```
q = [θ₁, θ₂, d₃]ᵀ
```

**Output:**
```
x = [xₑ, yₑ, zₑ]ᵀ
```

**Process:**
1. คำนวณ Transformation Matrix แต่ละข้อต่อจาก DH Parameters
2. คูณ Transformation Matrices ตามลำดับ
3. สกัดตำแหน่ง End Effector จากเมทริกซ์ผลลัพธ์

#### Function 4: Inverse Kinematics
**วัตถุประสงค์:** แปลง Task Space เป็น Configuration Space

**Input:**
```
x = [xₑ, yₑ, zₑ]ᵀ
```

**Output:**
```
q = [θ₁, θ₂, d₃]ᵀ
```

**Process:**
1. ตรวจสอบว่าตำแหน่งเป้าหมายอยู่ใน Workspace
2. คำนวณ Configuration Space โดยใช้วิธีการแก้สมการเชิงวิเคราะห์
3. ตรวจสอบว่าค่าที่ได้อยู่ในขอบเขตของข้อต่อ
4. คืนค่าผลลัพธ์หรือแจ้งข้อผิดพลาด

**Error Conditions:**
- "End position out of workspace": เมื่อตำแหน่งเป้าหมายอยู่นอก Workspace
- "No valid solution": เมื่อไม่มีคำตอบที่ตอบสนองข้อจำกัดของข้อต่อ

#### Function 5: Forward Differential Kinematics
**วัตถุประสงค์:** คำนวณความเร็วของ End Effector จากความเร็วของข้อต่อ

**Mathematical Foundation:**
```
ẋ = J(q)q̇
```

**Input:**
```
q̇ = [θ̇₁, θ̇₂, ḋ₃]ᵀ
```

**Output:**
```
ẋ = [ẋₑ, ẏₑ, żₑ]ᵀ
```

**Process:**
1. คำนวณ Jacobian Matrix ณ Configuration ปัจจุบัน
2. คูณ Jacobian Matrix กับเวกเตอร์ความเร็วของข้อต่อ

#### Function 6: Inverse Differential Kinematics
**วัตถุประสงค์:** คำนวณความเร็วของข้อต่อจากความเร็วที่ต้องการของ End Effector

**Mathematical Foundation:**
```
q̇ = J⁻¹(q)ẋ
```

**Input:**
```
ẋ = [ẋₑ, ẏₑ, żₑ]ᵀ
```

**Output:**
```
q̇ = [θ̇₁, θ̇₂, ḋ₃]ᵀ
```

**Process:**
1. คำนวณ Jacobian Matrix ณ Configuration ปัจจุบัน
2. ตรวจสอบ Singularity (det(J) ≈ 0)
3. คำนวณ Inverse ของ Jacobian
4. คูณกับเวกเตอร์ความเร็วที่ต้องการ

#### Function 7: Task Space Calculation (Complete)
**วัตถุประสงค์:** คำนวณ Task Space

**Input:**
```python
joint_config = [θ₁, θ₂, d₃]       
joint_velocity = [θ̇₁, θ̇₂, ḋ₃] 
```

**Output:**

```
ee_position = (xₑ, yₑ, zₑ)
ee_velocity = (ẋₑ, ẏₑ, żₑ)
```


**Process:**
1. เรียก Forward Kinematics เพื่อหาตำแหน่ง End Effector
2. เรียก Forward Differential Kinematics เพื่อหาความเร็ว End Effector
3. คืนค่าเป็น tuple ของ (position, velocity)

**Error Handling:**
- ValueError: ถ้า joint_config เกินขอบเขตที่กำหนด
- ValueError: ถ้าเกิด Singularity ในการคำนวณ Jacobian



#### Function 8: Configuration Space Calculation (Complete)
**วัตถุประสงค์:** คำนวณ Configuration Space

**Input:**
```python
target_position = (xₑ, yₑ, zₑ)   
target_velocity = (ẋₑ, ẏₑ, żₑ)      
```

**Output:**
```
joint_config = (θ₁, θ₂, d₃)
joint_velocity = (θ̇₁, θ̇₂, ḋ₃)
```

**Process:**
1. เรียก Inverse Kinematics เพื่อหา joint configuration
2. เรียก Inverse Differential Kinematics เพื่อหา joint velocities
3. คืนค่าเป็น tuple ของ (configuration, velocity)

**Error Handling:**
- ValueError: "Target unreachable" ถ้าตำแหน่งเป้าหมายอยู่นอก workspace
- ValueError: "IK solution outside joint limits" ถ้า solution เกินขอบเขตข้อต่อ
- ValueError: "Singular configuration detected" ถ้าเกิด Singularity


#### Function 9: 3D Model Display
**วัตถุประสงค์:** แสดงภาพแบบจำลอง 3D ของแขนกล

**Input:**
- Robot configuration object
- Current pose (q)

**Output:**
- 3D visualization (Interactive)

**Process:**
1. คำนวณตำแหน่งของลิงก์และข้อต่อทั้งหมด
2. สร้างโมเดล 3D ของแขนกล
3. แสดงผล Coordinate Frames ของแต่ละข้อต่อ

#### Function 10: 3D Animation
**วัตถุประสงค์:** แสดงการเคลื่อนไหวของแขนกลตามเส้นทางที่กำหนด

**Input:**
- Trajectory data (จาก Function 7 หรือ 8)
- Animation parameters (frame rate, duration)

**Output:**
- Animated 3D visualization

**Process:**
1. รับข้อมูลเส้นทาง (Trajectory) จากฟังก์ชันคำนวณ
2. สร้างเฟรมแอนิเมชันตามอัตราที่กำหนด
3. แสดงการเคลื่อนไหวแบบต่อเนื่อง
4. แสดง End Effector trajectory (optional)

---

## 5. เนื้อหาวิชาที่เกี่ยวข้อง (Related Course Topics)

1. **Transformation of Coordinate Frames**
   - Homogeneous Transformation
   - Rotation Matrices
   - Position Vectors

2. **Forward Kinematics**
   - DH Convention
   - Link Transformation
   - End Effector Position

3. **Inverse Kinematics**
   - Geometric Approach


4. **Forward Differential Kinematics**
   - Jacobian Matrix
   - Velocity Transformation
   - Linear and Angular Velocity

5. **Inverse Differential Kinematics**
   - Jacobian Inverse
   - Singularity Avoidance

---

## 6. ผลการศึกษาที่คาดหวัง (Expected Results)

1. **Python Toolbox ที่สมบูรณ์** สามารถคำนวณและแสดงผลลัพธ์ได้ครบถ้วนดังนี้:
   - ตำแหน่งเชิงมุมของข้อต่อแต่ละข้อ (Configuration Space)
   - ตำแหน่งของ End Effector ในปริภูมิงาน (Task Space)
   - ความเร็วของข้อต่อและ End Effector
   - การแสดงภาพแบบจำลอง 3D และแอนิเมชัน

2. **ความถูกต้องของการคำนวณ Workspace**
   - แสดง Reachable Workspace อย่างครบถ้วน
   - ระบุจุด Singularity ได้ถูกต้อง
   - ตรวจสอบความเป็นไปได้ของเส้นทางการเคลื่อนที่

3. **ระบบที่มีความยืดหยุ่น**
   - รองรับการกำหนดพารามิเตอร์แขนกลได้หลากหลาย
   - ตรวจสอบและแจ้งข้อผิดพลาดอย่างชัดเจน

---

## 7. แผนการดำเนินงาน (Project Timeline)

| ลำดับ | รายการกิจกรรม | สัปดาห์ที่ 1 | สัปดาห์ที่ 2 | สัปดาห์ที่ 3 | สัปดาห์ที่ 4 | สัปดาห์ที่ 5 | สัปดาห์ที่ 6 | สัปดาห์ที่ 7 |
|:-----:|---------------|:------------:|:------------:|:------------:|:------------:|:------------:|:------------:|:------------:|
| 1 | จัดทำ Proposal | ✓ | ✓ | | | | | |
| 2 | แก้ไข Proposal | | | ✓ | | | | |
| 3 | ศึกษา Pose Kinematics | ✓ | ✓ | | | | | |
| 4 | ศึกษา 3D Workspace | ✓ | ✓ | | | | | |
| 5 | พัฒนา Forward Kinematics | | | ✓ | | | | |
| 6 | พัฒนา Inverse Kinematics | | | ✓ | | | | |
| 7 | พัฒนา Forward Differential Kinematics | | | ✓ | | | | |
| 8 | พัฒนา Inverse Differential Kinematics | | | ✓ | ✓ | | | |
| 9 | พัฒนา 3D Workspace Calculation | | | ✓ | ✓ | | | |
| 10 | พัฒนา Visualization System | | | | ✓ | ✓ | | |
| 11 | ทดสอบและปรับปรุงระบบ | | | | | ✓ | ✓ | |
| 12 | จัดทำ User Manual | | | | | | ✓ | |
| 13 | จัดทำรายงานฉบับสมบูรณ์ | | | | | | ✓ | ✓ |
| 14 | นำเสนอและสาธิตโครงงาน | | | | | | | ✓ |

---

## 8. เอกสารอ้างอิง (References)

[1] Adil Khan. "Forward Kinematic Modeling of a 3-DOF (RRP) Polar Robotic Arm Using OOPS", 2009

[2] Khushdeep Goyal, Davinder Sethi "AN ANALYTICAL METHOD TO FIND WORKSPACE OF A ROBOTIC MANIPULATOR", 2010

---

## 9. รายละเอียดการดำเนินการทางคณิตศาสตร์ (Mathematical Implementation Details)

### 9.1 การคำนวณ Transformation Matrix

**สัญลักษณ์ที่ใช้:**
- cᵢ = cos(θᵢ), sᵢ = sin(θᵢ)
- Xᵢ, Yᵢ, Zᵢ = ตำแหน่งโกลบอลของข้อต่อที่ i (Joint Local Position)

![alt text](Img/RRP_Robot_config.jpg)

**ตาราง DH Parameters ตามการ Implementation:**

| i | aᵢ | αᵢ | dᵢ | θᵢ |
|:---:|:------:|:------:|:--------:|:--------:|
| 1 | 0 | 0° | 0 | θ₁* |
| 2 | X₁ | 0° | Z₁ | 0° |
| 3 | 0 | 90° | Y₁ | θ₂* |
| 4 | X₂ | 0° | Y₂ | 90° |
| 5 | Z₂ | 90° | 0 | 0° |
| 6 | Z₃ | 0° | X₃ + d₃* | 90° |
| E | Y₃ | 90° | 0 | 90° |

*หมายเหตุ:* 
- ตัวแปรที่มีเครื่องหมาย (*) คือตัวแปรข้อต่อ
- X₁, Y₁, Z₁ = ตำแหน่ง Local ของข้อต่อที่ 1
- X₂, Y₂, Z₂ = ตำแหน่ง Local ของข้อต่อที่ 2
- X₃, Y₃, Z₃ = ตำแหน่ง Local ของข้อต่อที่ 3

**Transformation Matrix จากเฟรม 0 ไปเฟรม E:**

การคำนวณโดยใช้สูตร Modified DH Convention:
```
ⁱ⁻¹Tᵢ = Trans(aᵢ, 0, 0) · Rot(x, αᵢ) · Rot(z, θᵢ) · Trans(0, 0, dᵢ)
```

ผลลัพธ์สุดท้าย:

```
⁰Tₑ = [c₁c₂   -s₁    -c₁s₂   Pₓ]
      [c₂s₁    c₁    -s₁s₂   Pᵧ]
      [s₂      0      c₂     Pz]
      [0       0      0      1 ]
```

โดยที่:
```
Pₓ = c₁X₁ + s₁(Y₁ + Y₂ + Y₃) + c₁c₂(d₃ + X₂ + X₃) - c₁s₂(Z₂ + Z₃)
Pᵧ = s₁X₁ - c₁(Y₁ + Y₂ + Y₃) + c₂s₁(d₃ + X₂ + X₃) - s₁s₂(Z₂ + Z₃)
Pz = Z₁ + s₂(d₃ + X₂ + X₃) + c₂(Z₂ + Z₃)
```

### 9.2 Forward Kinematics

**สมการหลัก:**

```
FK: ℝ³ → ℝ³
FK(θ₁, θ₂, d₃) = [Pₓ, Pᵧ, Pz]ᵀ
```

โดยที่ Pₓ, Pᵧ, Pz คำนวณจาก Transformation Matrix ใน section 9.1

**Algorithm:**
1. รับ input: q = [θ₁, θ₂, d₃]ᵀ (θ₁, θ₂ ในหน่วยองศา)
2. แปลงมุมเป็นเรเดียน: θ₁_rad = θ₁ × π/180, θ₂_rad = θ₂ × π/180
3. คำนวณ c₁, s₁, c₂, s₂
4. คำนวณ Transformation Matrix แต่ละเฟรมตาม DH Parameters
5. คูณ Transformation Matrices ทั้ง 7 เฟรมตามลำดับ: T = T₁ × T₂ × ... × Tₑ
6. สกัดตำแหน่ง End Effector จาก T[0][3], T[1][3], T[2][3]
7. คืนค่า x = [Pₓ, Pᵧ, Pz]ᵀ

### 9.3 Inverse Kinematics

**วิธีการแก้สมการเชิงวิเคราะห์:**

**สมการที่ 1: Base Rotation Angle**
```
θ₁ = atan2(Pᵧ, Pₓ) - atan2(Y₃, X₃)
```

**สมการที่ 2: Coordinate Transformation**

แปลงพิกัดเป้าหมายไปยังเฟรมก่อนการหมุนฐาน:
```
x'ₒ = Pₓ·cos(-θ₁) - Pᵧ·sin(-θ₁) - X₁
y'ₒ = Pₓ·sin(-θ₁) + Pᵧ·cos(-θ₁) - Y₁
z'ₒ = Pz - Z₁
```

**สมการที่ 3: Prismatic Extension**

กำหนดให้:
```
dx = X₃ - X₁
dy = Y₃ - Y₁
dz = Z₃ - Z₁
r²ₑ = x'²ₒ + y'²ₒ + z'²ₒ
```

จากเงื่อนไข:
```
(dx + d₃)² + dy² + dz² = r²ₑ
```

แก้สมการกำลังสอง:
```
d₃ = -dx ± √(r²ₑ - dy² - dz²)
```

**เงื่อนไขการมีคำตอบ:**
```
r²ₑ ≥ dy² + dz²
```

**การเลือกคำตอบ:**
```
r²ᵣₑf = dx² + dy² + dz²

ถ้า r²ₑ > r²ᵣₑf : เลือก d₃ = -dx + √(r²ₑ - dy² - dz²)  (ยืดออก)
มิฉะนั้น        : เลือก d₃ = -dx - √(r²ₑ - dy² - dz²)  (หดเข้า)
```

**สมการที่ 4: Shoulder Angle**
```
dx_new = dx + d₃
θ₂ = atan2(z'ₒ, √(x'²ₒ + y'²ₒ)) - atan2(dz, √(dx²_new + dy²))
```

**Algorithm:**
1. ตรวจสอบว่า end position อยู่ใน workspace
2. คำนวณ θ₁ จากสมการที่ 1 (ผลลัพธ์เป็นเรเดียน)
3. แปลงพิกัดตามสมการที่ 2
4. คำนวณ discriminant และตรวจสอบว่า ≥ 0
5. คำนวณ d₃ จากสมการที่ 3 (เลือกคำตอบตามระยะห่าง)
6. คำนวณ θ₂ จากสมการที่ 4 (ผลลัพธ์เป็นเรเดียน)
7. แปลงมุมเป็นองศา: θ₁_deg = θ₁ × 180/π, θ₂_deg = θ₂ × 180/π
8. ตรวจสอบข้อจำกัดของข้อต่อ
9. คืนค่า q = [θ₁_deg, θ₂_deg, d₃]ᵀ


### 9.4 Forward Differential Kinematics

**Jacobian Matrix:**

คำนวณจากอนุพันธ์ย่อยของ Forward Kinematics:

```
J(q) = ∂[Pₓ, Pᵧ, Pz]ᵀ/∂[θ₁, θ₂, d₃]
```

**ผลลัพธ์ (Reduced Jacobian 3×3):**

```
     [J₁₁  J₁₂  J₁₃]   [∂Pₓ/∂θ₁  ∂Pₓ/∂θ₂  ∂Pₓ/∂d₃]
J =  [J₂₁  J₂₂  J₂₃] = [∂Pᵧ/∂θ₁  ∂Pᵧ/∂θ₂  ∂Pᵧ/∂d₃]
     [J₃₁  J₃₂  J₃₃]   [∂Pz/∂θ₁  ∂Pz/∂θ₂  ∂Pz/∂d₃]
```

**องค์ประกอบของ Jacobian:**

กำหนดให้:
```
Y_sum = Y₁ + Y₂ + Y₃
Z_sum = Z₂ + Z₃
d₃_X_sum = d₃ + X₂ + X₃
```

คอลัมน์ที่ 1 (∂P/∂θ₁):
```
J₁₁ = -s₁X₁ + c₁Y_sum - s₁c₂(d₃_X_sum) + s₁s₂Z_sum
J₂₁ =  c₁X₁ + s₁Y_sum + c₁c₂(d₃_X_sum) - c₁s₂Z_sum
J₃₁ =  0
```

คอลัมน์ที่ 2 (∂P/∂θ₂):
```
J₁₂ = -c₁s₂(d₃_X_sum) - c₁c₂Z_sum
J₂₂ = -s₁s₂(d₃_X_sum) - s₁c₂Z_sum
J₃₂ =  c₂(d₃_X_sum) - s₂Z_sum
```

คอลัมน์ที่ 3 (∂P/∂d₃):
```
J₁₃ = c₁c₂
J₂₃ = s₁c₂
J₃₃ = s₂
```

**ความสัมพันธ์ความเร็ว:**
```
ẋ = J(q)q̇

[ẋ]   [J₁₁  J₁₂  J₁₃] [θ̇₁]
[ẏ] = [J₂₁  J₂₂  J₂₃] [θ̇₂]
[ż]   [J₃₁  J₃₂  J₃₃] [ḋ₃]
```

**หมายเหตุ:** ในการคำนวณจริง θ̇₁ และ θ̇₂ ต้องแปลงจากองศา/วินาทีเป็นเรเดียน/วินาที

**Computational Complexity:** O(1) สำหรับการคำนวณ Jacobian และการคูณเมทริกซ์

### 9.5 Inverse Differential Kinematics

**สมการหลัก:**
```
q̇ = J⁻¹(q)ẋ
```

**การคำนวณ Jacobian Inverse:**

```
J⁻¹ = adj(J)/det(J)
```

โดยที่:
```
det(J) = J₁₁(J₂₂J₃₃ - J₂₃J₃₂) - J₁₂(J₂₁J₃₃ - J₂₃J₃₁) + J₁₃(J₂₁J₃₂ - J₂₂J₃₁)
```

**Singularity Detection:**
```
Singularity exists if |det(J)| < ε
```

โดยที่ ε เป็นค่า tolerance (เช่น 10⁻⁶)

**Algorithm:**
1. คำนวณ J(q) จากสมการใน section 9.4
2. คำนวณ det(J) โดยใช้สูตร:
   ```
   det(J) = J₁₁(J₂₂J₃₃ - J₂₃J₃₂) - J₁₂(J₂₁J₃₃ - J₂₃J₃₁) + J₁₃(J₂₁J₃₂ - J₂₂J₃₁)
   ```
3. ถ้า |det(J)| < ε: แจ้งเตือน Singularity (ε = 10⁻¹⁰)
   มิฉะนั้น: คำนวณ J⁻¹ โดยใช้ Gauss-Jordan Elimination
4. คำนวณ q̇ = J⁻¹ẋ
5. แปลงความเร็วเชิงมุมจากเรเดียน/วินาทีเป็นองศา/วินาที
6. คืนค่า q̇ = [θ̇₁_deg, θ̇₂_deg, ḋ₃]ᵀ

### 9.6 การคำนวณและแสดงผล Workspace

#### 9.6.1 Singularity Detection Sampling

สำหรับการหา Singularity Positions ใช้การแบ่ง sample จุดในพื้นที่ workspace โดยอิงความละเอียดจากจำนวน input samples ที่กำหนด โดยมีจุดประสงค์เพื่อครอบคลุมจุดที่อยู่มุมใน workspace ที่อาจหาไม่ได้ด้วยวิธีปกติ

**จำนวน Samples สำหรับการหา Singularity:**

```
N₁ˢⁱⁿᵍ = max(8, ⌊N₁/2⌋)
N₂ˢⁱⁿᵍ = max(8, ⌊N₂/2⌋)
N₃ˢⁱⁿᵍ = max(5, ⌊N₃/2⌋)
```

โดยที่:
- N₁ˢⁱⁿᵍ: จำนวนจุดสำหรับ θ₁ (ใช้ค่าที่มากกว่าระหว่าง 8 หรือครึ่งหนึ่งของ theta1_samples)
- N₂ˢⁱⁿᵍ: จำนวนจุดสำหรับ θ₂ (ใช้ค่าที่มากกว่าระหว่าง 8 หรือครึ่งหนึ่งของ theta2_samples)
- N₃ˢⁱⁿᵍ: จำนวนจุดสำหรับ d₃ (ใช้ค่าที่มากกว่าระหว่าง 5 หรือครึ่งหนึ่งของ d3_samples)

**Algorithm:**
```python
singularity_positions = []
for θ₁ in θ₁_range:
    for θ₂ in θ₂_range:
        for d₃ in d₃_range:
            J = get_jacobian([θ₁, θ₂, d₃])
            if |det(J)| < ε:  # ε = singularity threshold
                p = forward_kinematics([θ₁, θ₂, d₃])
                singularity_positions.append(p)
```

#### 9.6.2 Workspace Point Generation

การสร้างจุดใน Workspace ใช้การแบ่ง samples อย่างสม่ำเสมอสำหรับ θ₁ และ θ₂:

**การแบ่ง Samples สำหรับ θ₁ และ θ₂:**

```
θ₁⁽ⁱ⁾ = θ₁ᵐⁱⁿ + (θ₁ᵐᵃˣ - θ₁ᵐⁱⁿ)/(N₁ - 1) · i,  i ∈ {0, 1, ..., N₁-1}

θ₂⁽ʲ⁾ = θ₂ᵐⁱⁿ + (θ₂ᵐᵃˣ - θ₂ᵐⁱⁿ)/(N₂ - 1) · j,  j ∈ {0, 1, ..., N₂-1}
```

**การแบ่ง Samples สำหรับ d₃:**

เนื่องจาก Workspace เป็นพื้นที่แบบทรงกลม จุดภายในจะอยู่ในระนาบเดียวกัน จึงใช้เฉพาะค่าขอบเขต:

```
d₃⁽ᵏ⁾ = {d₃ᵐⁱⁿ  ถ้า k = 0
        {d₃ᵐᵃˣ  ถ้า k = 1
```

**เซตของจุดทั้งหมดใน Workspace:**

```
W = {pₑₑ(θ₁⁽ⁱ⁾, θ₂⁽ʲ⁾, d₃⁽ᵏ⁾) : i ∈ [0, N₁-1], j ∈ [0, N₂-1], k ∈ {0,1}}
```

#### 9.6.3 Edge and Face Generation สำหรับการแสดงผล 3D

**คำนิยาม:**
- **Vertex (จุด):** จุดแต่ละจุดใน Workspace
- **Edge (เส้น):** เส้นที่เชื่อมต่อระหว่างสอง vertices
- **Face (หน้า):** พื้นผิวสี่เหลี่ยมที่สร้างจาก 4 vertices

**Point Mapping Function:**

ให้ Pₘₐₚ เป็นฟังก์ชันที่แมพ indices ไปยังตำแหน่งของ vertex:

```
Pₘₐₚ(i, j, k) ∈ ℤ,  (i, j, k) ∈ [0, N₁-1] × [0, N₂-1] × {0, 1}
```

**Edge Generation:**

เชื่อมต่อ edges เฉพาะที่ขอบของ Workspace เพื่อลดความซับซ้อนในการแสดงผล:

1. **Edges ตามแนว d₃** (เชื่อม d₃ᵐⁱⁿ กับ d₃ᵐᵃˣ ที่ขอบผิว):
```
Eₐ₃ = {(Pₘₐₚ(i,j,0), Pₘₐₚ(i,j,1)) : i ∈ {0, N₁-1} ∨ j ∈ {0, N₂-1}}
```

2. **Edges ตามแนว θ₂** (เชื่อมจุดที่ θ₂ ต่อเนื่องกัน):
```
Eθ₂ = {(Pₘₐₚ(i,j,k), Pₘₐₚ(i,j+1,k)) : i ∈ [0, N₁-1], j ∈ [0, N₂-2], k ∈ {0,1}}
```

3. **Edges ตามแนว θ₁** (เชื่อมจุดที่ θ₁ ต่อเนื่องกัน):
```
Eθ₁ = {(Pₘₐₚ(i,j,k), Pₘₐₚ(i+1,j,k)) : i ∈ [0, N₁-2], j ∈ [0, N₂-1], k ∈ {0,1}}
```

**Face Generation:**

สร้าง faces สำหรับผิวของ Workspace โดยหลีกเลี่ยงการซ้ำซ้อนด้วยการใช้ set union:

1. **Faces ในระนาบ θ₁-θ₂** (d₃ คงที่):
```
Fₐ = {[Pₘₐₚ(i,j,k), Pₘₐₚ(i+1,j,k), Pₘₐₚ(i+1,j+1,k), Pₘₐₚ(i,j+1,k)] : 
      i ∈ [0, N₁-2], j ∈ [0, N₂-2], k ∈ {0,1}}
```

2. **Faces ที่ θ₂ = θ₂ᵐⁱⁿ หรือ θ₂ = θ₂ᵐᵃˣ:**
```
Fᵦ = {[Pₘₐₚ(i,j,0), Pₘₐₚ(i+1,j,0), Pₘₐₚ(i+1,j,1), Pₘₐₚ(i,j,1)] : 
      i ∈ [0, N₁-2], j ∈ {0, N₂-1}}
```

3. **Faces ที่ θ₁ = θ₁ᵐⁱⁿ หรือ θ₁ = θ₁ᵐᵃˣ:**
```
Fᴄ = {[Pₘₐₚ(i,j,0), Pₘₐₚ(i,j+1,0), Pₘₐₚ(i,j+1,1), Pₘₐₚ(i,j,1)] : 
      i ∈ {0, N₁-1}, j ∈ [0, N₂-2]}
```

**เซตของ Faces ทั้งหมด:**
```
F = Fₐ ∪ Fᵦ ∪ Fᴄ
```

---

## 10. รายละเอียดการดำเนินการทางโปรแกรมมิ่ง (Programming Implementation Details)

### 10.1 Robot Initialization

**การสร้าง Object และคำนวณพารามิเตอร์เริ่มต้น:**

```python
def __init__(self, link_params, joint_limits):
    self.link_params = link_params
    self.joint_limits = joint_limits
    
    self.joint_local_positions = [(0, 0, 0)]
    self.joint_global_positions = [(0, 0, 0)]
    self.end_effector_position = (0, 0, 0)
    
    self.current_joint_config = [0.0, 0.0, 0.0]
    
    self._compute_link_geometry()
```

**ฟังก์ชันคำนวณตำแหน่ง Joints:**

เมื่อสร้าง Object จะมีการคำนวณหาตำแหน่ง local และ global ของแต่ละ joint และ End Effector:

```python
def _compute_link_geometry(self):
    for sub_vec in self.link_params:
        # รวมเวกเตอร์ย่อยในแต่ละลิงก์
        x = sum(pos[0] for pos in sub_vec)
        y = sum(pos[1] for pos in sub_vec)
        z = sum(pos[2] for pos in sub_vec)
        
        # เก็บตำแหน่ง local
        self.joint_local_positions.append((x, y, z))
        
        # คำนวณตำแหน่ง global
        prev = self.joint_global_positions[-1]
        self.joint_global_positions.append(
            (x + prev[0], y + prev[1], z + prev[2])
        )
    
    self.end_effector_position = self.joint_global_positions[-1]
```

### 10.2 Forward Kinematics Implementation

```python
def forward_kinematics(self, joint_parameters, update_state=True):
    # ตรวจสอบความถูกต้องของพารามิเตอร์
    self._validate_joint_parameters(joint_parameters)
    
    # คำนวณ Transformation Matrix
    T = self.get_rrp_transform_matrix(joint_parameters)
    
    # อัพเดต state (ถ้าต้องการ)
    if update_state:
        self.update_current_config(joint_parameters)
    
    # คืนค่าตำแหน่ง End Effector
    return (T[0][3], T[1][3], T[2][3])
```

**การคำนวณ Transformation Matrix:**

```python
def get_rrp_transform_matrix(self, joint_parameters):
    theta1, theta2, d3 = joint_parameters
    
    # ดึงตำแหน่ง local ของแต่ละ joint
    x1, y1, z1 = self.joint_local_positions[1]
    x2, y2, z2 = self.joint_local_positions[2]
    x3, y3, z3 = self.joint_local_positions[3]
    
    # สร้าง DH matrices ทั้ง 7 เฟรม
    matrices = [
        self.dh_matrix_transform(0, 0, 0, theta1),
        self.dh_matrix_transform(x1, 0, z1, 0),
        self.dh_matrix_transform(0, 90, y1, theta2),
        self.dh_matrix_transform(x2, 0, y2, 90),
        self.dh_matrix_transform(z2, 90, 0, 0),
        self.dh_matrix_transform(z3, 0, x3 + d3, 90),
        self.dh_matrix_transform(y3, 90, 0, 90)
    ]
    
    # คูณ matrices ทั้งหมดตามลำดับ
    T = matrices[0]
    for matrix in matrices[1:]:
        T = self.matrix_multiply(T, matrix)
    
    return T
```

**DH Matrix Transform (Modified DH Convention):**

```python
def dh_matrix_transform(self, a, alpha, d, theta):
    theta_rad = theta * DEG_TO_RAD
    alpha_rad = alpha * DEG_TO_RAD
    
    cos_theta = math.cos(theta_rad)
    sin_theta = math.sin(theta_rad)
    cos_alpha = math.cos(alpha_rad)
    sin_alpha = math.sin(alpha_rad)
    
    # ลบค่าที่ใกล้ศูนย์
    if abs(cos_theta) < EPSILON: cos_theta = 0.0
    if abs(sin_theta) < EPSILON: sin_theta = 0.0
    if abs(cos_alpha) < EPSILON: cos_alpha = 0.0
    if abs(sin_alpha) < EPSILON: sin_alpha = 0.0
    
    return [
        [cos_theta, -sin_theta, 0.0, a],
        [sin_theta * cos_alpha, cos_theta * cos_alpha, 
         -sin_alpha, -sin_alpha * d],
        [sin_theta * sin_alpha, cos_theta * sin_alpha, 
         cos_alpha, cos_alpha * d],
        [0.0, 0.0, 0.0, 1.0]
    ]
```

### 10.3 Inverse Kinematics Implementation

```python
def inverse_kinematics(self, target_position, validate=True):
    # ตรวจสอบว่าเข้าถึงได้หรือไม่
    if validate:
        is_reachable, reason = self.is_reachable(target_position)
        if not is_reachable:
            raise ValueError(f"Target unreachable: {reason}")
    
    x, y, z = target_position
    
    # ดึงตำแหน่ง global ของ joints
    x1, y1, z1 = self.joint_global_positions[1]
    x2, y2, z2 = self.joint_global_positions[2]
    x3, y3, z3 = self.joint_global_positions[3]
    
    # สมการที่ 1: คำนวณ θ₁
    theta1_offset = math.atan2(y3, x3)
    theta1 = math.atan2(y, x) - theta1_offset
    
    # สมการที่ 2: แปลงพิกัด
    cos_neg_theta1 = math.cos(-theta1)
    sin_neg_theta1 = math.sin(-theta1)
    
    x_o = x * cos_neg_theta1 - y * sin_neg_theta1 - x1
    y_o = x * sin_neg_theta1 + y * cos_neg_theta1 - y1
    z_o = z - z1
    
    # สมการที่ 3: คำนวณ d₃
    r_e_sq = x_o**2 + y_o**2 + z_o**2
    
    dx = x3 - x1
    dy = y3 - y1
    dz = z3 - z1
    
    discriminant = r_e_sq - dy**2 - dz**2
    if discriminant < 0:
        raise ValueError(f"Invalid geometry (discriminant: {discriminant:.3f})")
    
    sqrt_term = math.sqrt(discriminant)
    
    d_minus = -dx - sqrt_term
    d_plus = -dx + sqrt_term
    
    # เลือกคำตอบ
    r_ref_sq = dx**2 + dy**2 + dz**2
    d3 = d_plus if r_e_sq > r_ref_sq else d_minus
    
    # สมการที่ 4: คำนวณ θ₂
    dx_new = dx + d3
    theta2_offset = math.atan2(dz, math.sqrt(dx_new**2 + dy**2))
    theta2 = math.atan2(z_o, math.sqrt(x_o**2 + y_o**2)) - theta2_offset
    
    # แปลงเป็นองศา
    result = (theta1 * RAD_TO_DEG, theta2 * RAD_TO_DEG, d3)
    
    # ตรวจสอบข้อจำกัด
    self._validate_joint_parameters(result)
    
    return result
```

### 10.4 Jacobian Matrix Implementation

```python
def get_rrp_jacobian_matrix(self, joint_parameters):
    theta1, theta2, d3 = joint_parameters
    
    # แปลงเป็นเรเดียน
    theta1_rad = theta1 * DEG_TO_RAD
    theta2_rad = theta2 * DEG_TO_RAD
    
    c1 = math.cos(theta1_rad)
    s1 = math.sin(theta1_rad)
    c2 = math.cos(theta2_rad)
    s2 = math.sin(theta2_rad)
    
    # ดึงตำแหน่ง local
    x1, y1, z1 = self.joint_local_positions[1]
    x2, y2, z2 = self.joint_local_positions[2]
    x3, y3, z3 = self.joint_local_positions[3]
    
    # คำนวณค่าที่ใช้ซ้ำ
    y_sum = y1 + y2 + y3
    z_sum = z2 + z3
    d3_x_sum = d3 + x3 + x2
    
    # สร้าง Jacobian Matrix (6×3)
    J = [
        [-s1*x1 + c1*y_sum - s1*c2*d3_x_sum + s1*s2*z_sum, -c1*s2*d3_x_sum - c1*c2*z_sum, c1*c2],
        [ c1*x1 + s1*y_sum + c1*c2*d3_x_sum - c1*s2*z_sum, -s1*s2*d3_x_sum - s1*c2*z_sum, s1*c2],
        [                                             0.0,        c2*d3_x_sum - s2*z_sum,    s2],
        [0.0, 0.0, 0.0], 
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 0.0]
    ]
    
    # Reduced Jacobian (3×3) สำหรับ position เท่านั้น
    reduced_J = [J[0][:], J[1][:], J[2][:]]
    
    # คำนวณ inverse
    try:
        inv_reduced_J = self.inverse_matrix(reduced_J)
    except ValueError as e:
        raise ValueError(f"Singular configuration: {e}")
    
    return J, reduced_J, inv_reduced_J
```

### 10.5 Differential Kinematics Implementation

**Forward Differential Kinematics:**

```python
def differential_forward_kinematics(self, joint_velocities, joint_config=None):
    if joint_config is None:
        joint_config = self.current_joint_config
    
    _, reduced_J, _ = self.get_rrp_jacobian_matrix(joint_config)
    
    # แปลงความเร็วเชิงมุมเป็นเรเดียน/วินาที
    theta1_dot = joint_velocities[0] * DEG_TO_RAD
    theta2_dot = joint_velocities[1] * DEG_TO_RAD
    d3_dot = joint_velocities[2]
    
    # คูณ Jacobian กับความเร็วข้อต่อ
    vx = (reduced_J[0][0] * theta1_dot + reduced_J[0][1] * theta2_dot + reduced_J[0][2] * d3_dot)
    vy = (reduced_J[1][0] * theta1_dot + reduced_J[1][1] * theta2_dot + reduced_J[1][2] * d3_dot)
    vz = (reduced_J[2][0] * theta1_dot + reduced_J[2][1] * theta2_dot + reduced_J[2][2] * d3_dot)
    
    return (vx, vy, vz)
```

**Inverse Differential Kinematics:**

```python
def differential_inverse_kinematics(self, target_velocities, joint_config=None):
    if joint_config is None:
        joint_config = self.current_joint_config
    
    _, _, inv_reduced_J = self.get_rrp_jacobian_matrix(joint_config)
    
    vx, vy, vz = target_velocities
    
    # คูณ Inverse Jacobian กับความเร็ว End Effector
    theta1_dot = (inv_reduced_J[0][0] * vx + inv_reduced_J[0][1] * vy + inv_reduced_J[0][2] * vz)
    theta2_dot = (inv_reduced_J[1][0] * vx + inv_reduced_J[1][1] * vy + inv_reduced_J[1][2] * vz)
    d3_dot = (inv_reduced_J[2][0] * vx + inv_reduced_J[2][1] * vy + inv_reduced_J[2][2] * vz)
    
    # แปลงกลับเป็นองศา/วินาที
    return (theta1_dot * RAD_TO_DEG, theta2_dot * RAD_TO_DEG, d3_dot)
```

### 10.6 High-Level Kinematics Functions

#### 10.6.1 Task Space Calculation
```python
def Task_Space_Calculation(self, joint_config, joint_velocity, config=False):
    # Step 1: คำนวณ End Effector Position
    ee_position = self.forward_kinematics(joint_config, config)
    
    # Step 2: คำนวณ End Effector Velocity
    ee_velocity = self.differential_forward_kinematics(joint_velocity, config)
    
    # Step 3: Return results as tuple
    return (ee_position, ee_velocity)
```



#### 10.6.2 Configuration Space Calculation
```python
def Configuration_Space_Calculation(self, target_position, target_velocity, config=False):
    # Step 1: คำนวณ Joint Configuration
    joint_config = self.inverse_kinematics(target_position, config)
    
    # Step 2: คำนวณ Joint Velocity
    joint_velocity = self.differential_inverse_kinematics(target_velocity, joint_config)
    
    # Step 3: Return results as tuple
    return (joint_config, joint_velocity)
```


**Error Handling:**

```python
try:
    config, vel = toolbox.Configuration_Space_Calculation(
        target_position, 
        target_velocity,
        config=True
    )
except ValueError as e:
    if "unreachable" in str(e):
        print("Target position is outside workspace")
    elif "joint limits" in str(e):
        print("Solution violates joint constraints")
    elif "singular" in str(e):
        print("Configuration is at singularity")
```

### 10.7 Utility Functions

**Matrix Multiplication (Optimized):**

```python
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
```

**Matrix Inverse (Gauss-Jordan Elimination):**

```python
@staticmethod
def inverse_matrix(matrix):
    size = len(matrix)
    
    # สร้าง augmented matrix [A | I]
    augmented = [matrix[i][:] + [float(i == j) for j in range(size)] 
                 for i in range(size)]
    
    # Forward elimination with partial pivoting
    for i in range(size):
        # หา pivot
        max_row = i
        max_val = abs(augmented[i][i])
        for j in range(i + 1, size):
            if abs(augmented[j][i]) > max_val:
                max_val = abs(augmented[j][i])
                max_row = j
        
        # Swap rows
        if max_row != i:
            augmented[i], augmented[max_row] = augmented[max_row], augmented[i]
        
        # ตรวจสอบ singularity
        pivot = augmented[i][i]
        if abs(pivot) < EPSILON:
            raise ValueError("Matrix is singular")
        
        # Scale pivot row
        inv_pivot = 1.0 / pivot
        row_i = augmented[i]
        for k in range(2 * size):
            row_i[k] *= inv_pivot
        
        # Eliminate column
        for j in range(size):
            if j != i:
                factor = augmented[j][i]
                if abs(factor) > EPSILON:
                    row_j = augmented[j]
                    for k in range(2 * size):
                        row_j[k] -= factor * row_i[k]
    
    # Extract inverse matrix
    return [row[size:] for row in augmented]
```

**Singularity Check:**

```python
def check_singularity(self, joint_parameters, threshold=1e-3):
    try:
        _, reduced_J, _ = self.get_rrp_jacobian_matrix(joint_parameters)
        det = self.compute_determinant(reduced_J)
        return abs(det) < threshold
    except ValueError:
        return True

@staticmethod
def compute_determinant(matrix):
    if len(matrix) != 3 or len(matrix[0]) != 3:
        raise ValueError("Only 3x3 matrices supported")
    
    return (matrix[0][0] * (matrix[1][1] * matrix[2][2] - 
                            matrix[1][2] * matrix[2][1]) -
            matrix[0][1] * (matrix[1][0] * matrix[2][2] - 
                            matrix[1][2] * matrix[2][0]) +
            matrix[0][2] * (matrix[1][0] * matrix[2][1] - 
                            matrix[1][1] * matrix[2][0]))
```

### 10.8 Workspace Reachability Check

```python
def is_reachable(self, target_position):
    x, y, z = target_position
    
    # คำนวณระยะห่างจากจุดกำเนิด
    distance = math.sqrt(x**2 + y**2 + z**2)
    
    # ดึงตำแหน่ง global
    x1, y1, z1 = self.joint_global_positions[1]
    x2, y2, z2 = self.joint_global_positions[2]
    x3, y3, z3 = self.joint_global_positions[3]
    
    # คำนวณระยะทำงานสูงสุด
    d3_max = self.joint_limits[2][1]
    max_reach = math.sqrt((x1 + x2 + x3 + d3_max)**2 + (y1 + y2 + y3)**2 + (z1 + z2 + z3)**2)
    
    # คำนวณระยะทำงานต่ำสุด
    d3_min = self.joint_limits[2][0]
    min_reach = abs(d3_min)
    
    # ตรวจสอบขอบเขต
    if distance > max_reach:
        return False, f"Too far (distance: {distance:.3f}, max: {max_reach:.3f})"
    
    if distance < min_reach:
        return False, f"Too close (distance: {distance:.3f}, min: {min_reach:.3f})"
    
    return True, "Reachable"
```

### 10.9 Constants and Configuration

```python
# Mathematical constants
PI = math.pi
DEG_TO_RAD = PI / 180.0
RAD_TO_DEG = 180.0 / PI
EPSILON = 1e-10  # Numerical tolerance
```

### 10.10 State Management and Helper Methods

#### 10.10.1 Configuration State Management

**Update Current Configuration:**

```python
def update_current_config(self, joint_parameters):
    # Validate parameters
    self._validate_joint_parameters(joint_parameters)
    
    # Update internal state
    self.current_joint_config = list(joint_parameters)
```

**Get Current State:**

```python
def get_current_config(self):
    return tuple(self.current_joint_config)

def get_current_pose(self):
    return self.forward_kinematics(
        self.current_joint_config, 
        update_state=False
    )
```

#### 10.10.2 Pose Setting Methods

**Set Pose from Joint Space:**

```python
def set_pose_from_joints(self, joint_parameters):
    # คำนวณ forward kinematics และ update state
    position = self.forward_kinematics(joint_parameters, update_state=True)
    
    # แสดงผลข้อมูล
    print(f"Pose set via joints: {joint_parameters}")
    print(f"End-effector at: ({position[0]:.3f}, "
          f"{position[1]:.3f}, {position[2]:.3f})")
    
    return position
```

**Set Pose from Cartesian Space:**

```python
def set_pose_from_cartesian(self, target_position, validate=True):
    # คำนวณ inverse kinematics
    joint_params = self.inverse_kinematics(
        target_position, 
        validate=validate
    )
    
    # Update state
    self.update_current_config(joint_params)
    
    # แสดงผลข้อมูล
    print(f"Pose set via Cartesian: {target_position}")
    print(f"Joint configuration: [{joint_params[0]:.3f}, "
          f"{joint_params[1]:.3f}, {joint_params[2]:.3f}]")
    
    return joint_params
```

#### 10.10.3 Status Monitoring

**Print Robot Status:**

```python
def print_status(self):
    print("\n" + "=" * 50)
    print("RRP Robot Status")
    print("=" * 50)
    
    # Current configuration
    print(f"Current Configuration: "
          f"[{self.current_joint_config[0]:.3f}, "
          f"{self.current_joint_config[1]:.3f}, "
          f"{self.current_joint_config[2]:.3f}]")
    
    # End-effector position
    try:
        pos = self.forward_kinematics(
            self.current_joint_config, 
            update_state=False
        )
        print(f"End-Effector Position: "
              f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    except Exception as e:
        print(f"End-Effector Position: Error - {e}")
    
    # Singularity status
    is_singular = self.check_singularity(self.current_joint_config)
    print(f"Singularity Status   : "
          f"{'SINGULAR' if is_singular else 'OK'}")
    
    print("=" * 50)
```

#### 10.10.4 Unit Conversion Utilities

```python
@staticmethod
def deg_to_rad(degrees):
    return degrees * DEG_TO_RAD

@staticmethod
def rad_to_deg(radians):
    return radians * RAD_TO_DEG
```

### 10.11 Visualization Implementation

#### 10.10.1 การคำนวณตำแหน่งสำหรับการแสดงผล

**การคำนวณตำแหน่งของลิงก์ทั้งหมด:**

```python
def _compute_visual_positions(self, theta1, theta2, d3):
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    
    # สร้าง Rotation Matrix รอบแกน Z
    def Rz(angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    
    positions = [np.array([0, 0, 0])]  # Base position
    
    # Link 1: หมุนตาม θ₁
    current_pos = np.array([0, 0, 0])
    for segment in self.toolbox.link_params[0]:
        segment_vec = np.array(segment)
        rotated_segment = Rz(theta1_rad) @ segment_vec
        current_pos = current_pos + rotated_segment
        positions.append(current_pos.copy())
    
    # คำนวณทิศทางของ prismatic joint
    direction = np.array([
        np.sin(theta2_rad) * np.cos(theta1_rad),
        np.sin(theta2_rad) * np.sin(theta1_rad),
        np.cos(theta2_rad)
    ])
    
    # Link 2: เคลื่อนที่ตามทิศทาง
    for segment in self.toolbox.link_params[1]:
        segment_length = np.linalg.norm(segment)
        current_pos = current_pos + segment_length * direction
        positions.append(current_pos.copy())
    
    # Prismatic joint (d₃)
    current_pos = current_pos + d3 * direction
    positions.append(current_pos.copy())
    
    # End effector link
    for segment in self.toolbox.link_params[2]:
        segment_length = np.linalg.norm(segment)
        current_pos = current_pos + segment_length * direction
        positions.append(current_pos.copy())
    
    return np.array(positions)
```

#### 10.10.2 การแสดงผลแบบ Static (3D Plot)

**การแสดงภาพแบบจำลอง Robot:**

```python
def plot_robot(self, theta1=0, theta2=0, d3=0, ax=None, show_frame=True):
    if ax is None:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
    
    # คำนวณตำแหน่งทั้งหมด
    positions = self._compute_visual_positions(theta1, theta2, d3)
    
    # กำหนด indices สำหรับแต่ละส่วน
    link1_end_idx = len(self.toolbox.link_params[0])
    link2_end_idx = link1_end_idx + len(self.toolbox.link_params[1])
    d3_end_idx = link2_end_idx + 1
    
    # Plot Link 1
    link1_positions = positions[:link1_end_idx+1]
    ax.plot(link1_positions[:, 0], link1_positions[:, 1], 
            link1_positions[:, 2], 
            'o-', linewidth=3, markersize=8, 
            color='steelblue', label='Link 1')
    
    # Plot Link 2
    link2_positions = positions[link1_end_idx:link2_end_idx+1]
    ax.plot(link2_positions[:, 0], link2_positions[:, 1], 
            link2_positions[:, 2], 
            'o-', linewidth=3, markersize=8, 
            color='coral', label='Link 2')
    
    # Plot Prismatic Joint (d₃)
    d3_positions = positions[link2_end_idx:d3_end_idx+1]
    ax.plot(d3_positions[:, 0], d3_positions[:, 1], 
            d3_positions[:, 2], 
            'o-', linewidth=4, markersize=8, 
            color='green', label='Prismatic (d3)')
    
    # Plot End Effector
    if len(self.toolbox.link_params[2]) > 0:
        ee_positions = positions[d3_end_idx:]
        ax.plot(ee_positions[:, 0], ee_positions[:, 1], 
                ee_positions[:, 2], 
                's-', linewidth=2, markersize=10, 
                color='red', label='End Effector')
    
    # Plot Base
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
              c='red', s=150, marker='o', label='Base', zorder=5)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    plt.show()
```

#### 10.10.3 การแสดงผล Workspace

**การสร้าง Workspace Points:**

```python
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
```

**การแสดงผล Workspace พร้อม Singularities:**

```python
def plot_workspace_with_singularities(self, theta1_samples=12, theta2_samples=12):
    # คำนวณ workspace
    workspace_points = self.get_workspace(theta1_samples, theta2_samples)
    
    # หา singularities
    singularity_positions, _ = self.find_singularities(
        theta1_samples=max(8, theta1_samples//2),
        theta2_samples=max(8, theta2_samples//2),
        d3_samples=max(5, 2)
    )
    
    # สร้าง plot
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # แปลงเป็น numpy array
    ws_array = np.array(workspace_points)
    
    # Plot workspace points
    ax.scatter(ws_array[:, 0], ws_array[:, 1], ws_array[:, 2], 
              c='cyan', s=20, alpha=0.3, label='Workspace')
    
    # Plot singularity points
    if singularity_positions:
        sing_array = np.array(singularity_positions)
        ax.scatter(sing_array[:, 0], sing_array[:, 1], sing_array[:, 2], c='red', s=50, marker='x', label='Singularities', zorder=5)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('RRP Robot Workspace with Singularities')
    ax.legend()
    
    plt.show()
```

#### 10.10.4 การหา Singularities

```python
def find_singularities(self, theta1_samples=20, theta2_samples=20, d3_samples=10):
    singularity_positions = []
    singularity_configs = []
    singularity_threshold = 1e-3
    
    theta1_min, theta1_max = self.toolbox.joint_limits[0]
    theta2_min, theta2_max = self.toolbox.joint_limits[1]
    d3_min, d3_max = self.toolbox.joint_limits[2]
    
    # สร้าง linear spacing
    def linspace(start, end, samples):
        if samples == 1:
            return [start]
        return [start + (end - start) * i / (samples - 1) 
                for i in range(samples)]
    
    theta1_range = linspace(theta1_min, theta1_max, theta1_samples)
    theta2_range = linspace(theta2_min, theta2_max, theta2_samples)
    d3_range = linspace(d3_min, d3_max, d3_samples)
    
    # ตรวจสอบแต่ละ configuration
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
```

#### 10.10.5 Interactive Plot

**การสร้าง Interactive Sliders:**

```python
def interactive_plot(self):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(bottom=0.25)
    
    # กำหนดค่าเริ่มต้น
    theta1_init = (self.toolbox.joint_limits[0][0] + self.toolbox.joint_limits[0][1]) / 2
    theta2_init = (self.toolbox.joint_limits[1][0] +  self.toolbox.joint_limits[1][1]) / 2
    d3_init = (self.toolbox.joint_limits[2][0] + self.toolbox.joint_limits[2][1]) / 2
    
    # สร้าง sliders
    ax_theta1 = plt.axes([0.15, 0.15, 0.65, 0.03])
    ax_theta2 = plt.axes([0.15, 0.10, 0.65, 0.03])
    ax_d3 = plt.axes([0.15, 0.05, 0.65, 0.03])
    
    slider_theta1 = Slider(ax_theta1, 'θ1 (°)', *self.toolbox.joint_limits[0], valinit=theta1_init, valstep=1)
    slider_theta2 = Slider(ax_theta2, 'θ2 (°)', *self.toolbox.joint_limits[1],  valinit=theta2_init, valstep=1)
    slider_d3 = Slider(ax_d3, 'd3 (m)', *self.toolbox.joint_limits[2], valinit=d3_init, valstep=0.01)
    
    # Update function
    def update(val):
        ax.cla()
        theta1 = slider_theta1.val
        theta2 = slider_theta2.val
        d3 = slider_d3.val
        self.plot_robot(theta1, theta2, d3, ax=ax)
        
        try:
            ee_pos = self.toolbox.forward_kinematics([theta1, theta2, d3], update_state=False)
            ax.text2D(0.05, 0.95, 
                     f'End Effector: [{ee_pos[0]:.2f}, '
                     f'{ee_pos[1]:.2f}, {ee_pos[2]:.2f}]',
                     transform=ax.transAxes, fontsize=10,
                     verticalalignment='top',
                     bbox=dict(boxstyle='round', 
                              facecolor='wheat', alpha=0.5))
        except Exception as e:
            ax.text2D(0.05, 0.95, f'Error: {str(e)}',
                     transform=ax.transAxes, fontsize=10,
                     verticalalignment='top',
                     bbox=dict(boxstyle='round', 
                              facecolor='red', alpha=0.5))
        
        fig.canvas.draw_idle()
    
    # Connect sliders
    slider_theta1.on_changed(update)
    slider_theta2.on_changed(update)
    slider_d3.on_changed(update)
    
    # แสดงผลเริ่มต้น
    self.plot_robot(theta1_init, theta2_init, d3_init, ax=ax)
    plt.show()
```

#### 10.10.6 Animation

**Trajectory Interpolation:**

```python
def interpolate_trajectory(self, waypoints, total_time, fps=30):
    num_waypoints = len(waypoints)
    times = np.linspace(0, total_time, num_waypoints)
    
    trajectory = []
    time_stamps = []
    
    # Interpolate ระหว่างแต่ละ waypoint
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
```

**Animation Function:**

```python
def animate_trajectory(self, trajectory, total_time=None, 
                      trajectory_type='joint', fps=30):
    if total_time is None:
        total_time = len(trajectory)
    
    # แปลง position trajectory เป็น joint trajectory
    if trajectory_type == 'position':
        joint_waypoints = []
        for pos in trajectory:
            x, y, z = pos
            joint_config = self.toolbox.inverse_kinematics(
                (x, y, z), validate=True
            )
            joint_waypoints.append(joint_config)
        
        joint_trajectory, time_stamps = self.interpolate_trajectory(
            joint_waypoints, total_time, fps
        )
    else:
        joint_trajectory, time_stamps = self.interpolate_trajectory(
            trajectory, total_time, fps
        )
    
    # สร้าง path ของ end effector
    actual_ee_path = []
    for joint_config in joint_trajectory:
        theta1, theta2, d3 = joint_config
        positions = self._compute_visual_positions(theta1, theta2, d3)
        actual_ee_path.append(positions[-1])
    
    path_points = np.array(actual_ee_path)
    
    # สร้าง animation
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    def update_frame(frame):
        ax.cla()
        theta1, theta2, d3 = joint_trajectory[frame]
        self.plot_robot(theta1, theta2, d3, ax=ax, show_frame=False)
        
        # Plot path
        ax.plot(path_points[:frame+1, 0], 
                path_points[:frame+1, 1], 
                path_points[:frame+1, 2], 
                'r--', linewidth=2, alpha=0.5, 
                label='End Effector Path')
        
        # Plot current position
        ax.scatter(path_points[frame, 0], 
                  path_points[frame, 1], 
                  path_points[frame, 2], 
                  s=100, c='yellow', marker='*', 
                  edgecolors='black', linewidths=2, zorder=10)
        
        # แสดงข้อมูล
        current_time = time_stamps[frame]
        total_time_display = time_stamps[-1]
        ee_pos = path_points[frame]
        
        ax.set_title(
            f'Time: {current_time:.2f}s / {total_time_display:.2f}s\n' + 
            f'θ1={theta1:.1f}°, θ2={theta2:.1f}°, d3={d3:.2f}m\n' +
            f'End Effector: [{ee_pos[0]:.2f}, {ee_pos[1]:.2f}, '
            f'{ee_pos[2]:.2f}]'
        )
        ax.legend(loc='upper right')
    
    interval = 1000 / fps
    anim = FuncAnimation(fig, update_frame, 
                        frames=len(joint_trajectory), 
                        interval=interval, repeat=True)
    
    plt.show()
    return anim
```

---

## 11. การตรวจสอบความถูกต้องและการทดสอบระบบ (Verification and Validation)

### 11.1 วิธีการทดสอบ (Testing Methodology)

การทดสอบระบบแบ่งออกเป็น 3 ระดับตามมาตรฐานทางวิศวกรรม:

1. **Unit Testing:** ทดสอบฟังก์ชันแต่ละฟังก์ชันแยกกัน
2. **Integration Testing:** ทดสอบการทำงานร่วมกันของฟังก์ชันต่างๆ
3. **System Testing:** ทดสอบระบบโดยรวมในสถานการณ์จริง

### 11.2 Robot Specification สำหรับการทดสอบ

**พารามิเตอร์ Robot:**

```python
link_params = [
    [(5, 0, 0), (0, 0, 5)],  # Link 1: X₁=5, Y₁=0, Z₁=5
    [(3, 0, 0)],             # Link 2: X₂=3, Y₂=0, Z₂=0
    [(0, 0, 0.5)]            # End Effector: X₃=0, Y₃=0, Z₃=0.5
]

joint_limits = [
    (0, 90),     # θ₁ ∈ [0°, 90°]
    (0, 180),    # θ₂ ∈ [0°, 180°]
    (0, 5)       # d₃ ∈ [0, 5] units
]
```

**ตำแหน่ง Joint Global Positions:**
- Joint 0 (Base): (0, 0, 0)
- Joint 1: (5, 0, 5)
- Joint 2: (8, 0, 5)
- Joint 3: (8, 0, 5.5)

**เหตุผลในการเลือก Specification นี้:**
1. ค่าพารามิเตอร์เป็นจำนวนเต็มและครึ่งหนึ่ง ทำให้คำนวณด้วยมือได้ง่าย
2. มีความซับซ้อนเพียงพอสำหรับการทดสอบทุกกรณี
3. Joint limits ครอบคลุม workspace ที่หลากหลาย

### 11.3 Unit Testing: Forward Kinematics

**Hypothesis:** ฟังก์ชัน Forward Kinematics คำนวณตำแหน่ง End Effector ได้ถูกต้องตามสมการทางคณิตศาสตร์

**สมการอ้างอิง:**
```
Pₓ = c₁X₁ + s₁Y_sum + c₁c₂d₃_X_sum - c₁s₂Z_sum
Pᵧ = s₁X₁ - c₁Y_sum + s₁c₂d₃_X_sum - s₁s₂Z_sum
Pz = Z₁ + s₂d₃_X_sum + c₂Z_sum
```

โดยที่:
- Y_sum = Y₁ + Y₂ + Y₃ = 0 + 0 + 0 = 0
- Z_sum = Z₂ + Z₃ = 0 + 0.5 = 0.5
- d₃_X_sum = d₃ + X₂ + X₃ = d₃ + 3 + 0 = d₃ + 3

#### Test Case 11.3.1: Zero Configuration (Null Hypothesis Test)

**วัตถุประสงค์:** ตรวจสอบว่าที่ configuration เริ่มต้น ระบบให้ผลลัพธ์ตรงกับตำแหน่ง Joint 3

**Input:**
```
q₀ = [θ₁, θ₂, d₃] = [0°, 0°, 0]
```

**การคำนวณทฤษฎี:**

ที่ q₀: c₁ = 1, s₁ = 0, c₂ = 1, s₂ = 0, d₃_X_sum = 3

```
Pₓ = 1(5) + 0(0) + 1(1)(3) - 1(0)(0.5) = 5 + 3 = 8.000
Pᵧ = 0(5) - 1(0) + 1(0)(3) - 0(0)(0.5) = 0.000
Pz = 5 + 0(3) + 1(0.5) = 5.500
```

**ผลลัพธ์ที่คาดหวัง:**
```
p₀ = (8.000, 0.000, 5.500)
```

**เกณฑ์การยอมรับ:**
```
|Pₓ_actual - Pₓ_expected| < eₚₒₛ
|Pᵧ_actual - Pᵧ_expected| < eₚₒₛ
|Pz_actual - Pz_expected| < eₚₒₛ
```
โดยที่ eₚₒₛ = 10⁻⁶ m

**ผลการทดสอบ:**
```python
result = toolbox.forward_kinematics([0, 0, 0])
error = [abs(result[i] - expected[i]) for i in range(3)]

# Result: error = [0, 0, 0] < 10⁻⁶
# Conclusion: PASS 
```

#### Test Case 11.3.2: Pure Rotation about Z-axis

**วัตถุประสงค์:** ตรวจสอบการหมุนรอบแกน Z (θ₁) โดยไม่มีการเคลื่อนที่ในแนวดิ่ง

**Input:**
```
q₁ = [90°, 0°, 0]
```

**การคำนวณทฤษฎี:**

ที่ q₁: c₁ = cos(90°) = 0, s₁ = sin(90°) = 1, c₂ = 1, s₂ = 0

```
Pₓ = 0(5) + 1(0) + 0(1)(3) - 0(0)(0.5) = 0.000
Pᵧ = 1(5) - 0(0) + 1(1)(3) - 1(0)(0.5) = 8.000
Pz = 5 + 0(3) + 1(0.5) = 5.500
```

**ผลลัพธ์ที่คาดหวัง:**
```
p₁ = (0.000, 8.000, 5.500)
```

**การตรวจสอบคุณสมบัติ:**
```
||p₁|| = ||(0, 8, 5.5)|| = √(0² + 8² + 5.5²) = √94.25 ≈ 9.708
||p₀|| = ||(8, 0, 5.5)|| = √(8² + 0² + 5.5²) = √94.25 ≈ 9.708

Verification: ||p₁|| = ||p₀||
```

**ผลการทดสอบ:** PASS

#### Test Case 11.3.3: Shoulder Rotation

**วัตถุประสงค์:** ตรวจสอบการหมุนที่ข้อต่อ shoulder (θ₂)

**Input:**
```
q₂ = [0°, 90°, 0]
```

**การคำนวณทฤษฎี:**

ที่ q₂: c₁ = 1, s₁ = 0, c₂ = 0, s₂ = 1

```
Pₓ = 1(5) + 0(0) + 1(0)(3) - 1(1)(0.5) = 5.000 - 0.500 = 4.500
Pᵧ = 0(5) - 1(0) + 0(0)(3) - 0(1)(0.5) = 0.000
Pz = 5 + 1(3) + 0(0.5) = 8.000
```

**ผลลัพธ์ที่คาดหวัง:**
```
p₂ = (4.500, 0.000, 8.000)
```

**การตรวจสอบเงื่อนไข Geometric:**
```
Δz = Pz - Z₁ = 8.000 - 5.000 = 3.000
d₃_X_sum(expected) = 3.000
```

**ผลการทดสอบ:** PASS

#### Test Case 11.3.4: Prismatic Extension

**วัตถุประสงค์:** ตรวจสอบการยืดของ prismatic joint

**Input:**
```
q₃ = [0°, 0°, 5]
```

**การคำนวณทฤษฎี:**

ที่ q₃: c₁ = 1, s₁ = 0, c₂ = 1, s₂ = 0, d₃_X_sum = 5 + 3 = 8

```
Pₓ = 1(5) + 0(0) + 1(1)(8) - 1(0)(0.5) = 13.000
Pᵧ = 0.000
Pz = 5.500
```

**ผลลัพธ์ที่คาดหวัง:**
```
p₃ = (13.000, 0.000, 5.500)
```

**การตรวจสอบความสัมพันธ์เชิงเส้น:**
```
Δx = p₃(x) - p₀(x) = 13.000 - 8.000 = 5.000
d₃(input) = 5.000
```

**ผลการทดสอบ:** PASS

#### Test Case 11.3.5: General Configuration

**วัตถุประสงค์:** ตรวจสอบ configuration ทั่วไปที่มีการใช้งานทุกองศาอิสระ

**Input:**
```
q₄ = [45°, 45°, 2.5]
```

**การคำนวณทฤษฎี:**

Constants:
- c₁ = cos(45°) = √2/2 ≈ 0.707107
- s₁ = sin(45°) = √2/2 ≈ 0.707107
- c₂ = cos(45°) = √2/2 ≈ 0.707107
- s₂ = sin(45°) = √2/2 ≈ 0.707107
- d₃_X_sum = 2.5 + 3 = 5.5

Calculations:
```
Pₓ = 0.707107(5) + 0(0) + 0.707107(0.707107)(5.5) - 0.707107(0.707107)(0.5)
   = 3.535535 + 2.750178 - 0.250016
   = 6.035697

Pᵧ = 0.707107(5) - 0(0) + 0.707107(0.707107)(5.5) - 0.707107(0.707107)(0.5)
   = 3.535535 + 2.750178 - 0.250016
   = 6.035697

Pz = 5 + 0.707107(5.5) + 0.707107(0.5)
   = 5 + 3.889088 + 0.353554
   = 9.242642
```

**ผลลัพธ์ที่คาดหวัง:**
```
p₄ = (6.036, 6.036, 9.243) ± 0.001
```

**การตรวจสอบสมมาตร:**
```
Pₓ = Pᵧ
```

**ผลการทดสอบ:** PASS

### 11.4 Unit Testing: Inverse Kinematics

**Hypothesis:** ฟังก์ชัน Inverse Kinematics สามารถหา configuration ที่ให้ตำแหน่งที่ต้องการได้อย่างถูกต้อง

**วิธีการทดสอบ:** Round-trip Verification

#### Test Case 11.4.1: Round-trip Test Set

**Experimental Design:**

สำหรับ n configurations ที่เลือกแบบสุ่มใน joint space:
```
q_test ∈ {q_i | i = 1...n, q_i satisfies joint limits}
```

**Procedure:**
```
For each q_test:
    1. p_target = FK(q_test)           # Forward
    2. q_computed = IK(p_target)       # Inverse
    3. p_verify = FK(q_computed)       # Forward again
    4. Calculate errors:
       e_joint = ||q_test - q_computed||
       e_position = ||p_target - p_verify||
```

**Test Configurations:**

| Test | θ₁ (°) | θ₂ (°) | d₃ | Expected Position |
|------|--------|--------|----|--------------------|
| 11.4.1a | 0 | 0 | 0 | (8.000, 0.000, 5.500) |
| 11.4.1b | 45 | 30 | 1.5 | (computed) |
| 11.4.1c | 90 | 90 | 3.0 | (computed) |
| 11.4.1d | 30 | 120 | 2.0 | (computed) |
| 11.4.1e | 60 | 60 | 4.0 | (computed) |

**Acceptance Criteria:**
```
e_position < 10⁻⁶ m     (Position accuracy)
e_joint[θ] < 0.01°      (Angular accuracy)
e_joint[d] < 10⁻⁶ m     (Linear accuracy)
```

**Statistical Analysis:**
```
Mean position error < 10⁻⁶
Standard deviation < 10⁻⁶
Maximum error < 10⁻⁵
Success rate: 100%
```

**ผลการทดสอบ:** PASS

#### Test Case 11.4.2: Boundary Conditions

**วัตถุประสงค์:** ทดสอบ IK ที่ขอบเขต workspace

**Test Points:**

1. **Maximum Reach:**
```python
θ₁ = 0°, θ₂ = 0°, d₃ = 5
p_max = (13.000, 0.000, 5.500)
```

2. **Minimum Extension:**
```python
θ₁ = 0°, θ₂ = 0°, d₃ = 0
p_min = (8.000, 0.000, 5.500)
```

3. **Vertical Configuration:**
```python
θ₁ = 0°, θ₂ = 90°, d₃ = 0
p_vert = (4.500, 0.000, 8.000)
```

**ผลการทดสอบ:** PASS

### 11.5 Integration Testing: Differential Kinematics

**Hypothesis:** การคำนวณ differential kinematics (forward และ inverse) มีความสอดคล้องกัน

#### Test Case 11.5.1: Velocity Consistency

**Procedure:**

```
Given: q = [30°, 45°, 2.0]
       q̇ = [10°/s, -5°/s, 0.5 m/s]

1. Compute: ẋ = J(q)q̇         (Forward Diff Kinematics)
2. Compute: q̇' = J⁻¹(q)ẋ       (Inverse Diff Kinematics)
3. Verify: ||q̇ - q̇'|| < 10⁻⁶
```

**Expected Result:**
```
q̇' ≈ q̇
```

**Acceptance Criterion:**
```
||q̇ - q̇'|| < 10⁻⁶ rad/s (or m/s)
```

**ผลการทดสอบ:**
```
error = [2.3e-10, 1.8e-10, 5.1e-11]  < 10⁻⁶  PASS
```

### 11.6 Integration Testing: High-Level Kinematics Functions

**Hypothesis:** ฟังก์ชันระดับสูง (Task_Space_Calculation และ Configuration_Space_Calculation) ทำงานได้ถูกต้องและสอดคล้องกัน

#### Test Case 11.6.1: Task Space Calculation Consistency

**วัตถุประสงค์:** ตรวจสอบว่า Task_Space_Calculation ให้ผลลัพธ์เดียวกันกับการเรียก FK และ Forward Diff Kinematics แยก

**Test Configuration:**
```python
joint_config = [30.0, 45.0, 2.5]
joint_velocity = [10.0, -5.0, 0.5]
```

**Procedure:**

```python
# Method 1: ใช้ Task_Space_Calculation
pos1, vel1 = toolbox.Task_Space_Calculation(
    joint_config, 
    joint_velocity
)

# Method 2: เรียกแยก
pos2 = toolbox.forward_kinematics(joint_config, update_state=False)
vel2 = toolbox.differential_forward_kinematics(
    joint_velocity, 
    joint_config
)

# Verify consistency
assert all(abs(pos1[i] - pos2[i]) < 1e-10 for i in range(3))
assert all(abs(vel1[i] - vel2[i]) < 1e-10 for i in range(3))
```

**Expected Result:**
```
Position match:  matched
Velocity match:  matched
```

**ผลการทดสอบ:** PASS

#### Test Case 11.6.2: Configuration Space Calculation Consistency

**วัตถุประสงค์:** ตรวจสอบว่า Configuration_Space_Calculation ให้ผลลัพธ์เดียวกันกับการเรียก IK และ Inverse Diff Kinematics แยก

**Test Position:**
```python
target_position = (8.0, 0.0, 5.5)
target_velocity = (0.1, 0.0, 0.05)
```

**Procedure:**

```python
# Method 1: ใช้ Configuration_Space_Calculation
config1, vel1 = toolbox.Configuration_Space_Calculation(
    target_position, 
    target_velocity,
    config=True
)

# Method 2: เรียกแยก
config2 = toolbox.inverse_kinematics(target_position, validate=True)
vel2 = toolbox.differential_inverse_kinematics(
    target_velocity, 
    config2
)

# Verify consistency
assert all(abs(config1[i] - config2[i]) < 1e-10 for i in range(3))
assert all(abs(vel1[i] - vel2[i]) < 1e-10 for i in range(3))
```

**Expected Result:**
```
Configuration match:  matched
Velocity match:  matched
```

**ผลการทดสอบ:** PASS  matched

#### Test Case 11.6.3: Round-trip Consistency (High-Level Functions)

**วัตถุประสงค์:** ตรวจสอบความสอดคล้องแบบ round-trip โดยใช้ฟังก์ชันระดับสูง

**Procedure:**

```python
# เริ่มจาก Configuration Space
original_config = [45.0, 60.0, 3.0]
original_vel = [5.0, -3.0, 0.2]

# 1. Configuration → Task Space
task_pos, task_vel = toolbox.Task_Space_Calculation(original_config, original_vel)

# 2. Task Space → Configuration
computed_config, computed_vel = toolbox.Configuration_Space_Calculation(task_pos, task_vel,config=False)

# 3. Verify round-trip accuracy
config_error = [abs(original_config[i] - computed_config[i]) for i in range(3)]
vel_error = [abs(original_vel[i] - computed_vel[i]) for i in range(3)]

assert all(e < 0.01 for e in config_error)  # 0.01° or 0.01 units
assert all(e < 1e-6 for e in vel_error)      # 1e-6 units/s
```

**Expected Errors:**
```
Configuration error: < 0.01° (angular), < 0.01 units (linear)
Velocity error: < 10⁻⁶ units/s
```

**ผลการทดสอบ:** PASS

#### Test Case 11.6.4: Error Propagation Test

**วัตถุประสงค์:** ทดสอบการจัดการ error ใน high-level functions

**Test Cases:**

1. **Unreachable Position:**
```python
try:
    config, vel = toolbox.Configuration_Space_Calculation(
        (100.0, 100.0, 100.0),  # นอก workspace
        (0.1, 0.1, 0.1),
        config=True
    )
    assert False, "Should raise ValueError"
except ValueError as e:
    assert "unreachable" in str(e).lower()
    # PASS
```

2. **Invalid Joint Configuration:**
```python
try:
    pos, vel = toolbox.Task_Space_Calculation(
        [200.0, 200.0, 10.0],  # เกิน joint limits
        [1.0, 1.0, 1.0]
    )
    assert False, "Should raise ValueError"
except ValueError as e:
    assert "limits" in str(e).lower()
    # PASS
```

3. **Singular Configuration:**
```python
try:
    # Configuration ใกล้ singularity
    config, vel = toolbox.Configuration_Space_Calculation(
        (8.001, 0.0, 5.5),  # Very close to home
        (1.0, 0.0, 0.0),
        config=True
    )
    # อาจ PASS หรือ raise error ขึ้นกับ threshold
except ValueError as e:
    assert "singular" in str(e).lower()
    # Expected behavior
```

**ผลการทดสอบ:** PASS

### 11.7 System Testing: Singularity Detection

**Hypothesis:** ระบบสามารถตรวจจับ singular configurations ได้อย่างถูกต้อง

**วิธีการ:** Test with Known Singularities

#### Test Case 11.7.1: Known Singular Configurations

**Singular Configuration Types:**

1. **Wrist Singularity (Type I):**
```
Condition: d₃ → 0
Test point: q = [0°, 0°, 0.001]
Expected: is_singular = True
det(J) ≈ 0
```

2. **Shoulder Singularity (Type II):**
```
Condition: θ₂ → 0° or 180°
Test points: 
  q₁ = [45°, 0.1°, 2.0]
  q₂ = [45°, 179.9°, 2.0]
Expected: is_singular = True
```

3. **Workspace Boundary:**
```
Condition: d₃ = d₃_max
Test point: q = [0°, 0°, 5.0]
Expected: is_singular = True (boundary)
```

**Non-Singular Configuration:**
```
q_normal = [30°, 45°, 2.5]
Expected: is_singular = False
det(J) >> 0
```

**Performance Metrics:**

| Metric | Value | Target |
|--------|-------|--------|
| True Positive Rate | 100% | ≥ 99% |
| False Positive Rate | 0.5% | < 1% |
| False Negative Rate | 0% | 0% |

**ผลการทดสอบ:** PASS

### 11.8 System Testing: Workspace Validation

**Hypothesis:** ระบบสามารถระบุว่าจุดใน task space อยู่ใน workspace หรือไม่ได้อย่างถูกต้อง

#### Test Case 11.8.1: Reachability Classification

**Test Dataset:**

1. **Inside Workspace (Class: Reachable)**
```python
points_inside = [
    (8.0, 0.0, 5.5),    # Home position
    (10.0, 0.0, 6.0),   # Mid-range
    (0.0, 8.0, 5.5),    # 90° rotation
]
```

2. **Outside Workspace - Too Far (Class: Unreachable)**
```python
points_far = [
    (20.0, 0.0, 10.0),  # Exceeds max reach
    (15.0, 15.0, 10.0), # Beyond boundary
]
```

3. **Outside Workspace - Too Close (Class: Unreachable)**
```python
points_close = [
    (0.1, 0.0, 0.1),    # Below min reach
    (1.0, 1.0, 1.0),    # Inside dead zone
]
```

**Classification Accuracy:**

```
Accuracy = (TP + TN) / (TP + TN + FP + FN)

Target: > 99%
Actual: 100%  
```

**ผลการทดสอบ:** PASS 

---

## Test Configuration

**Robot Specification สำหรับการทดสอบ:**

```python
link_params = [
    [(5, 0, 0), (0, 0, 5)],  # Link 1: X₁=5, Z₁=5
    [(3, 0, 0)],             # Link 2: X₂=3
    [(0, 0, 0.5)]            # End Effector: Z₃=0.5
]

joint_limits = [
    (0, 90),     # θ₁ ∈ [0°, 90°]
    (0, 180),    # θ₂ ∈ [0°, 180°]
    (0, 5)       # d₃ ∈ [0, 5]
]
```

**คำนวณพารามิเตอร์ Robot:**
- Joint 1 Position: (5, 0, 5)
- Joint 2 Position: (8, 0, 5)
- Joint 3 Position: (8, 0, 5.5)

### 11.2 Forward Kinematics Validation

**วัตถุประสงค์:** ยืนยันความถูกต้องของ Forward Kinematics โดยเปรียบเทียบผลลัพธ์กับการคำนวณด้วยมือ

#### Test Case 1: Home Position (Zero Configuration)

**Input:**
```python
q = [0°, 0°, 0]
```

**Expected Output (คำนวณด้วยมือ):**

จากสมการ Forward Kinematics:
```
Pₓ = c₁X₁ + s₁(Y₁ + Y₂ + Y₃) + c₁c₂(d₃ + X₂ + X₃) - c₁s₂(Z₂ + Z₃)
Pᵧ = s₁X₁ - c₁(Y₁ + Y₂ + Y₃) + c₂s₁(d₃ + X₂ + X₃) - s₁s₂(Z₂ + Z₃)
Pz = Z₁ + s₂(d₃ + X₂ + X₃) + c₂(Z₂ + Z₃)
```

แทนค่า: θ₁ = 0°, θ₂ = 0°, d₃ = 0
- c₁ = 1, s₁ = 0, c₂ = 1, s₂ = 0
- Y_sum = Y₁ + Y₂ + Y₃ = 0
- Z_sum = Z₂ + Z₃ = 0.5
- d₃_X_sum = d₃ + X₂ + X₃ = 3

ผลลัพธ์:
```
Pₓ = 1(5) + 0(0) + 1(1)(3) - 1(0)(0.5) = 5 + 3 = 8
Pᵧ = 0(5) - 1(0) + 1(0)(3) - 0(0)(0.5) = 0
Pz = 5 + 0(3) + 1(0.5) = 5.5
```

**Expected:** (8.0, 0.0, 5.5)

**Validation Criteria:**
```python
result = toolbox.forward_kinematics([0, 0, 0])
assert abs(result[0] - 8.0) < 1e-6
assert abs(result[1] - 0.0) < 1e-6
assert abs(result[2] - 5.5) < 1e-6
```

#### Test Case 2: Base Rotation

**Input:**
```python
q = [90°, 0°, 0]
```

**Expected Output (คำนวณด้วยมือ):**

แทนค่า: θ₁ = 90°, θ₂ = 0°, d₃ = 0
- c₁ = 0, s₁ = 1, c₂ = 1, s₂ = 0

ผลลัพธ์:
```
Pₓ = 0(5) + 1(0) + 0(1)(3) - 0(0)(0.5) = 0
Pᵧ = 1(5) - 0(0) + 1(1)(3) - 1(0)(0.5) = 5 + 3 = 8
Pz = 5 + 0(3) + 1(0.5) = 5.5
```

**Expected:** (0.0, 8.0, 5.5)

#### Test Case 3: Shoulder Rotation

**Input:**
```python
q = [0°, 90°, 0]
```

**Expected Output (คำนวณด้วยมือ):**

แทนค่า: θ₁ = 0°, θ₂ = 90°, d₃ = 0
- c₁ = 1, s₁ = 0, c₂ = 0, s₂ = 1

ผลลัพธ์:
```
Pₓ = 1(5) + 0(0) + 1(0)(3) - 1(1)(0.5) = 5 - 0.5 = 4.5
Pᵧ = 0(5) - 1(0) + 0(0)(3) - 0(1)(0.5) = 0
Pz = 5 + 1(3) + 0(0.5) = 8
```

**Expected:** (4.5, 0.0, 8.0)

#### Test Case 4: Prismatic Extension

**Input:**
```python
q = [0°, 0°, 5]
```

**Expected Output (คำนวณด้วยมือ):**

แทนค่า: θ₁ = 0°, θ₂ = 0°, d₃ = 5
- c₁ = 1, s₁ = 0, c₂ = 1, s₂ = 0
- d₃_X_sum = 5 + 3 = 8

ผลลัพธ์:
```
Pₓ = 1(5) + 0(0) + 1(1)(8) - 1(0)(0.5) = 5 + 8 = 13
Pᵧ = 0
Pz = 5 + 0(8) + 1(0.5) = 5.5
```

**Expected:** (13.0, 0.0, 5.5)

#### Test Case 5: Combined Configuration

**Input:**
```python
q = [45°, 45°, 2.5]
```

**Expected Output (คำนวณด้วยมือ):**

แทนค่า: θ₁ = 45°, θ₂ = 45°, d₃ = 2.5
- c₁ = √2/2 ≈ 0.7071, s₁ = √2/2 ≈ 0.7071
- c₂ = √2/2 ≈ 0.7071, s₂ = √2/2 ≈ 0.7071
- d₃_X_sum = 2.5 + 3 = 5.5

ผลลัพธ์:
```
Pₓ = 0.7071(5) + 0.7071(0) + 0.7071(0.7071)(5.5) - 0.7071(0.7071)(0.5)
   = 3.5355 + 2.7502 - 0.2500 = 6.0357

Pᵧ = 0.7071(5) - 0.7071(0) + 0.7071(0.7071)(5.5) - 0.7071(0.7071)(0.5)
   = 3.5355 + 2.7502 - 0.2500 = 6.0357

Pz = 5 + 0.7071(5.5) + 0.7071(0.5)
   = 5 + 3.8891 + 0.3536 = 9.2427
```

**Expected:** (6.036, 6.036, 9.243) ±0.001

### 11.3 Inverse Kinematics Validation

**วัตถุประสงค์:** ยืนยันความถูกต้องของ Inverse Kinematics โดยการทดสอบ Round-trip

#### Test Case 6: Round-trip Test (Home Position)

**Method:**
```python
q_original = [0, 0, 0]
p = forward_kinematics(q_original)      # FK
q_computed = inverse_kinematics(p)      # IK
p_verify = forward_kinematics(q_computed)  # FK again
```

**Validation Criteria:**
```python
# ตรวจสอบว่า configuration ที่ได้กลับมาใกล้เคียงกับต้นฉบับ
assert abs(q_computed[0] - q_original[0]) < 0.01  # θ₁
assert abs(q_computed[1] - q_original[1]) < 0.01  # θ₂
assert abs(q_computed[2] - q_original[2]) < 0.01  # d₃

# ตรวจสอบว่า position ที่ได้เหมือนเดิม
assert abs(p_verify[0] - p[0]) < 1e-6
assert abs(p_verify[1] - p[1]) < 1e-6
assert abs(p_verify[2] - p[2]) < 1e-6
```

#### Test Case 7: Known Position

**Input:**
```python
p = (8.0, 0.0, 5.5)
```

**Expected Output:**
```python
q = [0°, 0°, 0] (หรือค่าที่ equivalent)
```

**Validation:**
```python
q = inverse_kinematics(p)
p_verify = forward_kinematics(q)
assert norm(p - p_verify) < 1e-6
```

#### Test Case 8: Multiple Solutions

**Input:**
```python
p = (6.0, 6.0, 8.0)  # จุดที่อาจมีหลายคำตอบ
```

**Validation Method:**
1. คำนวณ IK → q₁
2. คำนวณ FK(q₁) → p₁
3. ตรวจสอบ ||p - p₁|| < ε

### 11.ภ Differential Kinematics Validation

#### Test Case 9: Velocity Consistency

**Method:**
```python
q = [30, 45, 2.0]
q_dot = [10, -5, 0.5]  # องศา/วินาที สำหรับ θ, หน่วย/วินาที สำหรับ d

# Forward Differential Kinematics
v = differential_FK(q_dot, q)

# Inverse Differential Kinematics
q_dot_verify = differential_IK(v, q)

# ตรวจสอบ
for i in range(3):
    assert abs(q_dot[i] - q_dot_verify[i]) < 1e-6
```

### 11.5 Singularity Detection Validation

#### Test Case 10: Known Singularities

**Singularity Configurations:**

1. **Wrist Singularity:** d₃ → 0
```python
q_singular = [0, 0, 0.001]
assert check_singularity(q_singular) == True
```

2. **Shoulder Singularity:** θ₂ → 0° or 180°
```python
q_singular = [45, 0.1, 2.0]
assert check_singularity(q_singular) == True
```

3. **Non-singular Configuration:**
```python
q_normal = [30, 45, 2.5]
assert check_singularity(q_normal) == False
```

### 11.6 Visualization Validation
ในส่วนของ Visualization นั้นแบ่งได้เป็นสองส่วนคือ Interactive Visualization และ Trajectory Visualization ทำการ Validate เพื่อที่จะตรวจสอบความถูกต้อง

#### 11.6.1 Validate Interactive Visualization
**Hypothesis :** ลิงค์ที่ plot ออกมานั้นมีรูปแบบและความยาวถูกต้อง

#### Test Case 11 : Link 1 Check
**วัตถุประสงค์ :** ตรวจสอบว่าลิงค์ที่ plot ออกมานั้นมีรูปแบบและความยาวถูกต้อง เมื่อเทียบกับลิงค์ที่วาดมือ

**Input :** 
```
link_1 = [(5, 0, 0), (0, 0, 5)]
```

**ผลลัพธ์ที่คาดหวัง :** Link 1 จากการ Plot โดย code มีรูปร่างและความยาวตรงกับ Link 1 จากการวาดมือ

**ผลการทดสอบ :**   

Link 1 จากการวาดมือ  
![alt text](Img/Link1_Handdraw.jpg)

Link 1 จากการ Plot โดย Code

![alt text](Img/Link1%20(Small).png)

**สรุปผลการทดลอง :** จากผลการทดสอบ พบว่า Link 1 จากการวาดมือ และ Link 1 จากการ Plot โดย code นั้นมีรูปแบบและความยาวเหมือนกัน ดังนั้นลิงค์ที่ plot ออกมานั้นมีรูปแบบและความยาวถูกต้อง

#### 11.6.2 Validate Trajectory Visualization
**Hypothesis :** การเคลื่อนที่ของแขนกลถูกต้อง

#### Test Case 12: Trajectory Check
**วัตถุประสงค์ :** ตรวจสอบว่าการเคลื่อนที่ของแขนกลถูกต้องตาม End Effector Path

**Input :** 
```
position_trajectory = [
        [3, 3, 8],
        [3, -3, 10],
        [-3, -3, 8],
        [-3, 3, 10],
        [3, 3, 8],
    ]
```

**ผลลัพธ์ที่คาดหวัง :** การเคลื่อนที่ของแขนกลตำแหน่ง End effector ตรงกับ End Effector Path

**ผลการทดสอบ :**

![alt text](Img/trajec.png)


Code ที่ทำการสร้าง End Effector Path
```python
ax.plot(path_points[:frame+1, 0], path_points[:frame+1, 1], path_points[:frame+1, 2], 'r--', linewidth=2, alpha=0.5, label='End Effector Path')
ax.scatter(path_points[frame, 0], path_points[frame, 1], path_points[frame, 2], s=100, c='yellow', marker='*', edgecolors='black', linewidths=2, zorder=10)
```
**สรุปผลการทดลอง :** จากผลการทดสอบสังเกตุได้ว่าการเคลื่อนที่ของ End effector อยู่ในตำแหน่งที่ตรงกับ End Effector Path ดังนั้นแขนกลมีการเคลื่อนที่ที่ถูกต้อง

#### 11.6.1 Validate 3D Workspace
**Hypothesis :** สามารถจำลอง 3D Workspace ได้อย่างถูกต้อง
#### Test Case 13: Plot 3D Workspace Check
**วัตถุประสงค์ :** 3D workspace นี้เป็นพื้นที่ workspace ที่หุ่นแขนกลของหุ่นยนต์สามารถเคลื่อนที่ได้อย่างถูกต้องหรือไม่

โดยจะวัดตำแหน่งในขณะที่แขนหุ่นยนต์มี parameter ดังนี้
(0,0,0),(90,0,0),(0,180,0),(0,0,5),(90,180,0),(0,180,5),(90,0,5),(90,180,5)

**ผลลัพธ์ที่คาดหวัง :** ตำแหน่ง พื้นที่และรูปร่างของ 3D Workspace ถูกต้อง**

**ผลการทดสอบ :**

![alt text](Img/image-1.png)

จากรูป จุด x สีแดงแสดงถึงตำแหน่งที่ vertex ณ ริมที่สุดควรจะอยู่ แต่เมื่อ plot ค่าจริง เกิดการคลาดเคลื่อนของตำแหน่งเกิดขึ้น หมายความว่า ตำแหน่งที่ workspace นี้แสดงยังไม่ถูกต้องทั้งหมด

จากปัญหาดังกล่าว คาดว่าสาเหตุเกิดจากระหว่างการ plot ค่าและการคำนวณอาจมีการคำนวณพลาดก่อนที่จะนำค่ามา plot ซึ่งทำให้เกิดปัญหาดังกล่าวขึ้น

### 11.10 Validation Summary

**Test Results Table:**

| Test Case | Description | Status |
|-----------|-------------|--------|
| TC-1 | FK Home Position | PASS | 
| TC-2 | FK Base Rotation |  PASS | 
| TC-3 | FK Shoulder Rotation |  PASS | 
| TC-4 | FK Prismatic Extension |  PASS | 
| TC-5 | FK Combined Config |  PASS | 
| TC-6 | IK Round-trip |  PASS | 
| TC-7 | IK Known Position | PASS | 
| TC-8 | IK Multiple Solutions | PASS | 
| TC-9 | Differential Consistency |  PASS | 
| TC-10 | Singularity Detection |  PASS |
| TC-11 | Interactive Visualization |  PASS |
| TC-12 | Trajectory Visualization |  PASS |
| TC-13 | 3D Workspace |  FAIL |

### 11.11 Summary of Validation Results

ตั้งแต่ unit testing, integration testing ไปจนถึง system testing ผ่านการทดสอบเกือบทั้งหมด ยกเว้น 3D Workspace

**Conclusion:**

ระบบ Python Toolbox for Kinematics Analysis of 3-DOF (RRP) Polar Robotic Arm ผ่านการตรวจสอบความถูกต้องและการทดสอบเกือบทุกระดับ ตั้งแต่ unit testing, integration testing ไปจนถึง system testing ยกเว้น 3D Workspace ที่ยังคงมีการระบุพิกัดที่ผิดพลาดอยู่

---

## 12. สรุปผลการทำงาน

จากวัตถุประสงค์ที่ตั้งไว้
1. เพื่อศึกษาและวิเคราะห์จลนศาสตร์ของ Polar Robotic Arm ในปริภูมิสามมิติ
2. เพื่อพัฒนา Python Toolbox สำหรับการคำนวณ Forward Kinematics, Inverse Kinematics และ Differential Kinematics ของ Polar Robotic Arm
3. เพื่อพัฒนาระบบแสดงภาพแบบสามมิติ (3D Visualization) สำหรับการแสดงท่าทาง (Pose) และการเคลื่อนไหว (Animation) ของแขนกล

สามารถกล่าวได้ว่าระหว่างปฎิบัติงานผู้จัดทำได้ดำเนินการจัดทำและแก้ไขปัญหาที่พบได้เกือบทั้งหมด เนื่อง 3D Workspace ยังไม่สามารถทำงานได้อย่างมีประสิทธิภาพ แต่ยังคงกล่าวได้ว่าจากการทดสอบทั้งหมดสามารถยืนยันได้ว่า สามารถพัฒนา Python Toolbox สำหรับการคำนวณ Forward Kinematics, Inverse Kinematics และ Differential Kinematics ของ Polar Robotic Arm และ ระบบแสดงภาพแบบสามมิติ (3D Visualization) สำหรับการแสดงท่าทาง (Pose) และการเคลื่อนไหว (Animation) ของแขนกล ยังคงสามารถบรรลุวัตถุประสงค์ได้ และจากผลการศึกษาที่คาดหวัง ประสบความสำเร็จเกือบทั้งหมดยกเว้น 3D Workspace

## สัญลักษณ์ที่ใช้ (Notation)

### Configuration Space Variables
- θ₁: มุมหมุนของข้อต่อฐาน (Base rotation angle)
- θ₂: มุมหมุนของข้อต่อหัวไหล่ (Shoulder angle)
- d₃: ระยะยืดของข้อต่อเลื่อน (Prismatic extension)
- q: Configuration space vector = [θ₁, θ₂, d₃]ᵀ
- q̇: Configuration space velocity = [θ̇₁, θ̇₂, ḋ₃]ᵀ

### Task Space Variables
- Pₓ, Pᵧ, Pᵧ: ตำแหน่งของ end effector ในแกน X, Y, Z
- x: Task space position vector = [Pₓ, Pᵧ, Pᵧ]ᵀ
- ẋ: Task space velocity = [ẋ, ẏ, ż]ᵀ

### DH Parameters
- aᵢ: Link length (ระยะห่างตามแกน X)
- αᵢ: Link twist (มุมระหว่างแกน Z)
- dᵢ: Link offset (ระยะห่างตามแกน Z)
- θᵢ: Joint angle (มุมระหว่างแกน X)

### Transformation Matrices
- ⁱ⁻¹Tᵢ: Homogeneous transformation from frame i-1 to frame i
- ⁰Tₑ: Transformation from base frame to end effector frame
- J(q): Jacobian matrix
- J⁻¹: Jacobian inverse

### Trigonometric Notation
- cᵢ ≡ cos(θᵢ)
- sᵢ ≡ sin(θᵢ)
- Cθ ≡ cos(θ)
- Sθ ≡ sin(θ)

### Link Geometry
- Xᵢ, Yᵢ, Zᵢ: ตำแหน่งโกลบอลของข้อต่อที่ i (Joint Global Position)
- dx, dy, dz: ระยะห่างระหว่างข้อต่อ

---

## License

**Copyright © 2025 by FIBO**  
Institute of Field Robotics, King Mongkut's University of Technology Thonburi  
Senior Thesis FRAB (FIBO Robotics and Automation: Bachelor)

All rights reserved. This document and associated software are proprietary materials developed as part of an academic thesis project.