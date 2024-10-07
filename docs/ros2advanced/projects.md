---
layout: default
title: Small Assignment and Large Semester Project
nav_order: 4
has_children: true
permalink: /semester_project/
icon: material/home
hide:
  - toc
---

# Small Assignment and Large Semester Project

The purpose of the small assignment is to provide students with practical experience in ROS 2 and GitHub alongside the basic theoretical knowledge acquired in class. The small assignment can be completed in a **relatively short time**: an instructor can finish it in a few hours, and an average student can complete it in a few afternoons. Its length can be short, typically 30-100 lines of code per node.

In contrast, the large semester project takes a bit more time but allows for more interesting tasks and provides ample time to complete them. Moreover, good and excellent grades can only be achieved through this project.

Another way to earn grades is through midterm exams, but only modest grades can be obtained this way.

```mermaid
flowchart TD

K([Mandatory</br>small assignment]):::light ------> |unsuccessful| X0
K --> A([Signature]):::green 
A -->|Midterm direction| ZH1([Midterm 1]):::light
A --> |Semester project direction| N([Large semester project]):::light
PZH --> |unsuccessful| X1
ZH2 --> |unsuccessful| PZH([Make-up Midterm]):::light
ZH1 --> ZH2([Midterm 2]):::light
ZH2 --> |successful| OK1
PZH --> |successful| OK1
N ----> |successful| OK2
X0([Signature denied]):::red
X1([1, fail]):::red
OK2([4/5 grade]):::green
OK1([2/3 grade]):::green

classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff
classDef green fill:#138b7b,stroke:#152742,stroke-width:2px,color:#fff

```

## Deadlines and Semester Schedule

It's important to know that the small assignment is a **prerequisite for the signature**. Failure to register on GitHub or submit the assignment link early in the semester can result in an unsuccessful term. These are small tasks, but their completion is strictly monitored.

```mermaid
flowchart LR

H2([2nd occasion]) --- H2A([Github<br>registration])--- H2B([Start Copilot<br>registration])
H3([3rd occasion]) --- H3A([Submit assignment<br>Github link]) --- H3B([Copilot<br>registration complete])
H5([5th occasion]) --- H5A([Finalize small<br>assignment])
H7([7th occasion]) --- H7A([Midterm 1]) --- H7B([Submit large semester<br>Github link])
H10([10th occasion]) --- H10A([Midterm 2])
H13([13th occasion]) --- H13A([Make-up Midterm])
V2([Exam period 2nd week]) --- V2A([Finalize large<br>semester project])

classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff
classDef green fill:#138b7b,stroke:#152742,stroke-width:2px,color:#fff

class H2,H3,H5,H7,H10,H13,V2 white
class H2A,H2B,H3B,H7A,H7B,H10A,V2A light
class H3A,H5A,H13A red
```
