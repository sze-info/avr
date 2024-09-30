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

The purpose of the small assignment is for students to gain practical experience with ROS 2 and GitHub alongside the basic theoretical knowledge acquired in class. The small assignment can be completed in a relatively **short time**: an instructor can finish it in a few hours, and an average student can complete it in a few afternoons. Its length can be short, around 30-100 lines of code per node.

In contrast, the large semester project takes a bit more time but allows for much more interesting tasks and time. Moreover, good and excellent grades can only be achieved this way.

Another way to earn grades is through midterms (ZH), but only modest grades can be obtained this way.

```mermaid
flowchart TD

K([Mandatory</br>small assignment]):::light ------> |unsuccessful| X0
K --> A([Signature]):::green 
A -->|Midterm direction| ZH1([1st Midterm]):::light
A --> |Semester direction| N([Large Semester Project]):::light
PZH --> |unsuccessful| X1
ZH2 --> |unsuccessful| PZH([Make-up Midterm]):::light
ZH1 --> ZH2([2nd Midterm]):::light
ZH2 --> |successful| OK1
PZH --> |successful| OK1
N ----> |successful| OK2
X0([Signature Denied]):::red
X1([1, Fail]):::red
OK2([4/5 Grade]):::green
OK1([2/3 Grade]):::green

classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff
classDef green fill:#138b7b,stroke:#152742,stroke-width:2px,color:#fff

```

## Deadlines and Semester Schedule

It is important to know that the small assignment is a requirement for the signature. Missing the GitHub registration or the submission of the assignment link can result in an unsuccessful semester relatively early in the semester. These are small tasks, but their completion is strictly monitored.

```mermaid
flowchart LR

H2([2nd session]) --- H2A([Github<br>registration])--- H2B([Copilot registration<br>initiation])
H3([3rd session]) --- H3A([Assignment Github<br>link submission]) --- H3B([Copilot<br>registration complete])
H5([5th session]) --- H5A([Finalization of<br>small assignment])
H7([7th session]) --- H7A([1st Midterm]) --- H7B([Large Semester Project Github<br>link submission])
H10([10th session]) --- H10A([2nd Midterm])
H13([13th session]) --- H13A([Make-up Midterm])
V2([2nd week of exam period]) --- V2A([Finalization of<br>large semester project])


classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff
classDef green fill:#138b7b,stroke:#152742,stroke-width:2px,color:#fff

class H2,H3,H5,H7,H10,H13,V2 white
class H2A,H2B,H3B,H7A,H7B,H10A,V2A light
class H3A,H5A,H13A, red
```